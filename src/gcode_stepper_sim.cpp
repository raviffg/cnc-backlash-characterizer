//
// gcode_stepper_sim.cpp
// G-code G00/G01/G02 parser plus a 3-axis leadscrew backlash model.
//
// Purpose: quantify how much of the measured position error at a direction
// reversal comes from the TMC2209 microstep quantisation (1/16 on a 2 mm pitch
// lead, 200 full steps/rev => 0.000625 mm per microstep) versus mechanical
// deadband in the anti-backlash nut.
//
// The model is deliberately simple: each axis keeps a signed "lash state".
// Commanded motion opposite to the last direction has to consume the deadband
// before the carriage moves at all. That reproduces the classic hysteresis
// loop you see on an optical scale.
//
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Mechanics, measured on the benchtop router.
constexpr double kScrewPitchMm = 2.0;
constexpr double kFullStepsPerRev = 200.0;
constexpr int kMicrosteps = 16;
constexpr double kStepsPerMm =
    (kFullStepsPerRev * kMicrosteps) / kScrewPitchMm;  // 1600
constexpr double kMicrostepMm = 1.0 / kStepsPerMm;     // 0.000625 mm

struct AxisModel {
    double deadband_mm = 0.030;   // total lash, split either side of zero
    double last_dir = 0.0;        // sign of the last commanded move
    double lash_used = 0.0;       // 0..deadband_mm, consumed in current dir
    double position_mm = 0.0;     // modelled carriage position
};

struct Machine {
    AxisModel x{0.032, 0.0, 0.0, 0.0};
    AxisModel y{0.026, 0.0, 0.0, 0.0};
    AxisModel z{0.041, 0.0, 0.0, 0.0};
};

struct Move {
    enum Kind { Rapid, Feed, ArcCw, ArcCcw } kind = Rapid;
    double x = 0.0, y = 0.0, z = 0.0;
    double i = 0.0, j = 0.0;
    double feed = 0.0;
    bool has_x = false, has_y = false, has_z = false;
};

// --- G-code parsing -------------------------------------------------------

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool parseWord(const std::string& line, char letter, double& out) {
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (std::toupper(static_cast<unsigned char>(line[i])) != letter) continue;
        const char* start = line.c_str() + i + 1;
        char* end = nullptr;
        const double v = std::strtod(start, &end);
        if (end == start) return false;
        out = v;
        return true;
    }
    return false;
}

bool parseLine(const std::string& raw, Move& mv, Move::Kind& modal) {
    std::string line = upper(raw);
    // Strip comments and whitespace.
    const auto semi = line.find(';');
    if (semi != std::string::npos) line.erase(semi);
    const auto paren = line.find('(');
    if (paren != std::string::npos) line.erase(paren);
    if (line.empty()) return false;

    double g = -1.0;
    if (parseWord(line, 'G', g)) {
        switch (static_cast<int>(std::lround(g))) {
            case 0: mv.kind = Move::Rapid; break;
            case 1: mv.kind = Move::Feed; break;
            case 2: mv.kind = Move::ArcCw; break;
            case 3: mv.kind = Move::ArcCcw; break;
            default: return false;
        }
        modal = mv.kind;
    } else {
        // G is modal: a bare "X10 Y10" inherits the previous motion mode.
        mv.kind = modal;
    }

    mv.has_x = parseWord(line, 'X', mv.x);
    mv.has_y = parseWord(line, 'Y', mv.y);
    mv.has_z = parseWord(line, 'Z', mv.z);
    parseWord(line, 'I', mv.i);
    parseWord(line, 'J', mv.j);
    parseWord(line, 'F', mv.feed);
    return true;
}

// --- Backlash model -------------------------------------------------------

// Advance one axis by `delta` mm of commanded travel. Returns the realised
// carriage motion. Lash is consumed before any motion is transmitted.
double stepAxis(AxisModel& ax, double delta) {
    if (delta == 0.0) return 0.0;

    const double dir = (delta > 0.0) ? 1.0 : -1.0;
    if (dir != ax.last_dir) {
        // Direction change: lash has to be taken up again from the far side.
        ax.lash_used = 0.0;
        ax.last_dir = dir;
    }

    double remaining = std::fabs(delta);
    double moved = 0.0;

    if (ax.lash_used < ax.deadband_mm) {
        const double slack = ax.deadband_mm - ax.lash_used;
        const double eat = std::min(slack, remaining);
        ax.lash_used += eat;
        remaining -= eat;
    }

    moved = remaining * dir;

    // Quantise to the microstep grid: the driver cannot do better than this.
    const double target = ax.position_mm + moved;
    const double quantised = std::round(target * kStepsPerMm) / kStepsPerMm;
    const double realised = quantised - ax.position_mm;
    ax.position_mm = quantised;

    return realised;
}

// Flatten an arc into short chords. Chord tolerance 0.005 mm is well under the
// microstep size, so quantisation dominates the error budget as intended.
void flattenArc(const Move& mv, const double start[3], std::vector<std::array<double, 3>>& out) {
    const double cx = start[0] + mv.i;
    const double cy = start[1] + mv.j;
    const double r = std::hypot(start[0] - cx, start[1] - cy);
    if (r < 1e-9) return;

    double a0 = std::atan2(start[1] - cy, start[0] - cx);
    double a1 = std::atan2(mv.y - cy, mv.x - cx);
    if (mv.kind == Move::ArcCw && a1 >= a0) a1 -= 2.0 * kPi;
    if (mv.kind == Move::ArcCcw && a1 <= a0) a1 += 2.0 * kPi;

    const double sweep = a1 - a0;
    const double cos_arg = std::min(1.0, std::max(-1.0, 1.0 - 0.005 / r));
    const double max_chord = 2.0 * std::acos(cos_arg);
    int segments =
        static_cast<int>(std::ceil(std::fabs(sweep) / std::max(max_chord, 1e-4)));
    segments = std::min(std::max(segments, 4), 2048);

    for (int s = 1; s <= segments; ++s) {
        const double a = a0 + sweep * (static_cast<double>(s) / segments);
        out.push_back({cx + r * std::cos(a), cy + r * std::sin(a), mv.z});
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.gcode>\n", argv[0]);
        return 2;
    }

    std::FILE* fp = std::fopen(argv[1], "r");
    if (!fp) {
        std::perror("fopen");
        return 1;
    }

    Machine m;
    double commanded[3] = {0.0, 0.0, 0.0};
    double total_error = 0.0;
    double worst_error = 0.0;
    long reversals = 0;
    long moves = 0;
    double prev_dir[3] = {0.0, 0.0, 0.0};

    char buf[512];
    Move::Kind modal = Move::Rapid;
    while (std::fgets(buf, sizeof(buf), fp)) {
        Move mv;
        if (!parseLine(buf, mv, modal)) continue;

        std::vector<std::array<double, 3>> waypoints;
        const double target[3] = {
            mv.has_x ? mv.x : commanded[0],
            mv.has_y ? mv.y : commanded[1],
            mv.has_z ? mv.z : commanded[2],
        };

        if (mv.kind == Move::ArcCw || mv.kind == Move::ArcCcw) {
            Move arc = mv;
            if (!arc.has_z) arc.z = commanded[2];
            flattenArc(arc, commanded, waypoints);
        } else {
            waypoints.push_back({target[0], target[1], target[2]});
        }

        for (const auto& wp : waypoints) {
            const double cmd_delta[3] = {
                wp[0] - commanded[0], wp[1] - commanded[1], wp[2] - commanded[2]};
            AxisModel* axes[3] = {&m.x, &m.y, &m.z};
            for (int a = 0; a < 3; ++a) {
                const double dir = (cmd_delta[a] > 0.0) ? 1.0 : (cmd_delta[a] < 0.0 ? -1.0 : 0.0);
                if (dir != 0.0 && prev_dir[a] != 0.0 && dir != prev_dir[a]) ++reversals;
                if (dir != 0.0) prev_dir[a] = dir;
                stepAxis(*axes[a], cmd_delta[a]);
            }
            commanded[0] = wp[0];
            commanded[1] = wp[1];
            commanded[2] = wp[2];
            ++moves;
        }

        const double err[3] = {
            commanded[0] - m.x.position_mm,
            commanded[1] - m.y.position_mm,
            commanded[2] - m.z.position_mm,
        };
        const double e = std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
        total_error += e;
        worst_error = std::max(worst_error, e);
    }
    std::fclose(fp);

    std::printf("microstep_mm   : %.6f\n", kMicrostepMm);
    std::printf("waypoints      : %ld\n", moves);
    std::printf("reversals      : %ld\n", reversals);
    std::printf("final_error_mm : %.4f %.4f %.4f\n",
                commanded[0] - m.x.position_mm,
                commanded[1] - m.y.position_mm,
                commanded[2] - m.z.position_mm);
    std::printf("worst_error_mm : %.4f\n", worst_error);
    if (moves > 0) {
        std::printf("mean_error_mm  : %.4f\n", total_error / static_cast<double>(moves));
    }
    return 0;
}
