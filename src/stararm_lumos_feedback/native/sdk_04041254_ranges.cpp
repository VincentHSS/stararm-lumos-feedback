// Source compatibility layer for the SHA-pinned SDK 04041254 Linux x86-64 core.
// The Python loader MUST verify the reviewed native hash before calling this ABI.
// No controller, CAN socket, motor register operation or background thread is used.
//
// The reviewed core dynamically reads this exported table through its GOT in
// Motor::get_limit_param, MIT encoding and motor-state decoding. Only DM4340's
// vMax changes (8 -> 10); every other table value, gain and algorithm is retained.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#if !defined(__x86_64__) || !defined(__GLIBCXX__) || _GLIBCXX_USE_CXX11_ABI != 1
#error "This compatibility layer supports only the reviewed Linux x86-64 libstdc++ ABI"
#endif

#define STARARM_EXPORT __attribute__((visibility("default")))

namespace startouch::damiao_motor {
struct LimitParam {
    double pMax, vMax, tMax;
};

// External linkage and default visibility are required for ELF interposition.
extern STARARM_EXPORT const std::array<LimitParam, 13> MOTOR_LIMIT_PARAMS = {{
    {12.5, 50, 5}, {12.5, 30, 10}, {12.5, 50, 10},
    {12.5, 10, 28},  // DM4340: hardware RID22 is 10 for the bound J1-J3.
    {12.5, 10, 28}, {12.5, 45, 20}, {12.5, 45, 40},
    {12.5, 45, 54}, {12.5, 25, 200}, {12.5, 20, 200},
    {12.5, 280, 1}, {12.5, 45, 10}, {12.5, 45, 10},
}};
}  // namespace startouch::damiao_motor

namespace {
using startouch::damiao_motor::LimitParam;
using MotorType = std::uint8_t;
constexpr const char* kTableSymbol = "_ZN9startouch12damiao_motor18MOTOR_LIMIT_PARAMSE";
constexpr std::array<LimitParam, 13> kExpected = {{
    {12.5, 50, 5}, {12.5, 30, 10}, {12.5, 50, 10},
    {12.5, 10, 28}, {12.5, 10, 28}, {12.5, 45, 20},
    {12.5, 45, 40}, {12.5, 45, 54}, {12.5, 25, 200},
    {12.5, 20, 200}, {12.5, 280, 1}, {12.5, 45, 10}, {12.5, 45, 10},
}};

struct MITParam {
    double kp, kd, position, velocity, torque;
};
struct MotorState {
    double position, velocity, torque;
    int mos_temperature, rotor_temperature, motor_id, error_code;
    bool feedback_valid;
};
// Reviewed MotorC1 only initializes these 128 bytes, including an empty map.
// No map elements are inserted, so there are no allocations to destroy.
struct alignas(8) MotorStorage {
    std::array<unsigned char, 128> bytes{};
};

static_assert(sizeof(void*) == 8 && sizeof(double) == 8 && sizeof(int) == 4);
static_assert(std::numeric_limits<double>::is_iec559 && sizeof(MotorType) == 1);
static_assert(sizeof(LimitParam) == 24 && alignof(LimitParam) == 8);
static_assert(sizeof(kExpected) == 312 && sizeof(startouch::damiao_motor::MOTOR_LIMIT_PARAMS) == 312);
static_assert(std::is_standard_layout_v<MITParam> && sizeof(MITParam) == 40);
static_assert(offsetof(MITParam, kd) == 8 && offsetof(MITParam, position) == 16);
static_assert(offsetof(MITParam, velocity) == 24 && offsetof(MITParam, torque) == 32);
static_assert(std::is_standard_layout_v<MotorState> && sizeof(MotorState) == 48);
static_assert(offsetof(MotorState, velocity) == 8 && offsetof(MotorState, torque) == 16);
static_assert(offsetof(MotorState, mos_temperature) == 24 && offsetof(MotorState, rotor_temperature) == 28);
static_assert(offsetof(MotorState, motor_id) == 32 && offsetof(MotorState, error_code) == 36);
static_assert(offsetof(MotorState, feedback_valid) == 40);
static_assert(sizeof(MotorStorage) == 128 && alignof(MotorStorage) == 8);
static_assert(sizeof(std::vector<std::uint8_t>) == 24);

using GetLimits = LimitParam (*)(MotorType);
using ConstructMotor = void (*)(MotorStorage*, MotorType, unsigned int, unsigned int);
using Pack = std::vector<std::uint8_t> (*)(MotorType, const MITParam&);
using Decode = MotorState (*)(const MotorStorage&, const std::vector<std::uint8_t>&);

thread_local char last_error[384] = "Verification has not run";
std::mutex verification_mutex;
void* verified_core = nullptr;  // Retain the verified native DSO for the process lifetime.
std::string verified_path;

template <typename Function>
Function symbol(void* handle, const char* name) {
    dlerror();
    void* result = dlsym(handle, name);
    const char* error = dlerror();
    if (error || !result) throw std::runtime_error(std::string("Missing native symbol: ") + name);
    return reinterpret_cast<Function>(result);
}

void require(bool condition, int motor_type, const char* detail) {
    if (!condition) {
        throw std::runtime_error("Motor type " + std::to_string(motor_type) + ": " + detail);
    }
}

unsigned quantize(double value, double maximum, unsigned bits) {
    const double minimum = -maximum;
    return static_cast<unsigned>((value - minimum) / (maximum - minimum) * ((1u << bits) - 1));
}

double expand(unsigned code, double maximum, unsigned bits) {
    return static_cast<double>(code) / ((1u << bits) - 1) * (maximum - (-maximum)) + (-maximum);
}

std::vector<std::uint8_t> expected_command(const MITParam& command, const LimitParam& limits) {
    const unsigned p = quantize(command.position, limits.pMax, 16);
    const unsigned v = quantize(command.velocity, limits.vMax, 12);
    const unsigned t = quantize(command.torque, limits.tMax, 12);
    const unsigned kp = static_cast<unsigned>(command.kp / 500.0 * 4095);
    const unsigned kd = static_cast<unsigned>(command.kd / 5.0 * 4095);
    return {
        static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
        static_cast<std::uint8_t>(v >> 4), static_cast<std::uint8_t>((v << 4) | (kp >> 8)),
        static_cast<std::uint8_t>(kp), static_cast<std::uint8_t>(kd >> 4),
        static_cast<std::uint8_t>((kd << 4) | (t >> 8)), static_cast<std::uint8_t>(t),
    };
}

void verify_impl(const char* native_path) {
    if (!native_path || native_path[0] != '/') throw std::runtime_error("An absolute native core path is required");

    // Comparing &MOTOR_LIMIT_PARAMS with dlsym alone would be insufficient:
    // our own exported reference could also be preempted by an earlier DSO.
    void* table = symbol<void*>(RTLD_DEFAULT, kTableSymbol);
    Dl_info owner{}, local{};
    if (!dladdr(table, &owner) || !dladdr(reinterpret_cast<void*>(&verify_impl), &local)
        || owner.dli_fbase != local.dli_fbase) {
        throw std::runtime_error("Range table is not bound to this compatibility library; restart in a fresh process");
    }
    if (std::memcmp(table, kExpected.data(), sizeof(kExpected)) != 0) {
        throw std::runtime_error("Exported range table differs from the reviewed compatibility table");
    }

    if (verified_core && verified_path != native_path) {
        throw std::runtime_error("A different native core was already verified in this process");
    }
    if (!verified_core) {
        void* prior = dlopen(native_path, RTLD_NOW | RTLD_NOLOAD | RTLD_LOCAL);
        if (prior) {
            dlclose(prior);
            throw std::runtime_error("Native core was loaded before range verification; restart in a fresh process");
        }
        verified_core = dlopen(native_path, RTLD_NOW | RTLD_LOCAL);
        if (!verified_core) {
            const char* error = dlerror();
            throw std::runtime_error(std::string("Cannot load native codecs: ") + (error ? error : "unknown error"));
        }
        verified_path = native_path;
    }
    const auto limits = symbol<GetLimits>(verified_core,
        "_ZN9startouch12damiao_motor5Motor15get_limit_paramENS0_9MotorTypeE");
    const auto construct = symbol<ConstructMotor>(verified_core,
        "_ZN9startouch12damiao_motor5MotorC1ENS0_9MotorTypeEjj");
    const auto pack = symbol<Pack>(verified_core,
        "_ZN9startouch12damiao_motor16CanPacketEncoder21pack_mit_control_dataENS0_9MotorTypeERKNS0_8MITParamE");
    const auto decode = symbol<Decode>(verified_core,
        "_ZN9startouch12damiao_motor16CanPacketDecoder22parse_motor_state_dataERKNS0_5MotorERKSt6vectorIhSaIhEE");

    constexpr std::array<double, 7> fractions = {-1, -0.5, -0.125, 0, 0.125, 0.5, 1};
    constexpr std::array<unsigned, 8> velocity_codes = {0, 1, 1023, 2047, 2048, 2252, 4094, 4095};
    constexpr std::array<unsigned, 8> position_codes = {0, 1, 16383, 32767, 32768, 49151, 65534, 65535};
    for (MotorType type = 0; type < kExpected.size(); ++type) {
        const auto wanted = kExpected[type];
        const auto actual = limits(type);
        require(actual.pMax == wanted.pMax && actual.vMax == wanted.vMax && actual.tMax == wanted.tMax,
                type, "native table getter does not use the compatibility ranges");
        MotorStorage motor;
        construct(&motor, type, 1, 0x11);
        for (double fraction : fractions) {
            const MITParam command{fraction == 1 ? 500.0 : 83.5, fraction == -1 ? 5.0 : 2.125,
                                   fraction * wanted.pMax, fraction * wanted.vMax, fraction * wanted.tMax};
            require(pack(type, command) == expected_command(command, wanted), type,
                    "actual MIT encoder disagrees at signed range/zero cases");
        }
        for (double speed : {-1.0, 1.0}) {
            const MITParam command{0, 0, 0, speed, 0};
            require(pack(type, command) == expected_command(command, wanted), type,
                    "actual MIT encoder disagrees at nonzero physical velocity");
        }
        for (std::size_t i = 0; i < velocity_codes.size(); ++i) {
            const unsigned p = position_codes[i], v = velocity_codes[i], t = velocity_codes[7 - i];
            const std::vector<std::uint8_t> frame = {
                0x11, static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
                static_cast<std::uint8_t>(v >> 4), static_cast<std::uint8_t>((v << 4) | (t >> 8)),
                static_cast<std::uint8_t>(t), 41, 53,
            };
            const auto state = decode(motor, frame);
            require(std::isfinite(state.position) && std::isfinite(state.velocity) && std::isfinite(state.torque),
                    type, "actual decoder returned nonfinite mechanical state");
            require(std::abs(state.position - expand(p, wanted.pMax, 16)) < 1e-12
                    && std::abs(state.velocity - expand(v, wanted.vMax, 12)) < 1e-12
                    && std::abs(state.torque - expand(t, wanted.tMax, 12)) < 1e-12,
                    type, "actual state decoder disagrees at signed endpoints/quantization cases");
            require(state.feedback_valid && state.motor_id == 1 && state.error_code == 1
                    && state.mos_temperature == 41 && state.rotor_temperature == 53,
                    type, "actual state decoder status/ABI mismatch");
        }
    }
}
}  // namespace

extern "C" STARARM_EXPORT int stararm_ranges_verify(const char* native_path) noexcept {
    try {
        std::lock_guard<std::mutex> lock(verification_mutex);
        verify_impl(native_path);
        last_error[0] = '\0';
        return 0;
    } catch (const std::exception& error) {
        std::snprintf(last_error, sizeof(last_error), "%s", error.what());
    } catch (...) {
        std::snprintf(last_error, sizeof(last_error), "Unknown native range verification failure");
    }
    std::fprintf(stderr, "stararm motor ranges: %s\n", last_error);
    return 1;
}

extern "C" STARARM_EXPORT const char* stararm_ranges_error() noexcept {
    return last_error;
}
