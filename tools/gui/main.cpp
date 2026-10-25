// PID++ Control Lab: interactive tuning and visualization tool.
//
// Usage:
//   pidpp_control_lab              open the GUI
//   pidpp_control_lab --headless   run every plant preset without a window
//                                  (used as a CI smoke test)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include "lab_simulation.hpp"

namespace {

struct AppState {
    lab::Settings settings{lab::defaults_for(lab::PlantModel::FirstOrder)};
    lab::Trace trace{};
    std::string autotune_message{};
    bool auto_run{true};
};

constexpr const char* anti_windup_items =
    "None\0Clamping (dynamic)\0Conditional integration\0Back calculation\0";
constexpr const char* derivative_items = "On error\0On measurement\0";
constexpr const char* tuning_rule_items =
    "Ziegler-Nichols P\0Ziegler-Nichols PI\0Ziegler-Nichols PID\0Tyreus-Luyben PI\0"
    "Tyreus-Luyben PID\0Pessen integral\0Some overshoot PID\0No overshoot PID\0";

void apply_app_style() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 14.0F;
    style.ChildRounding = 12.0F;
    style.FrameRounding = 8.0F;
    style.PopupRounding = 8.0F;
    style.GrabRounding = 8.0F;
    style.ScrollbarRounding = 8.0F;
    style.TabRounding = 8.0F;
    style.WindowPadding = ImVec2{16.0F, 16.0F};
    style.FramePadding = ImVec2{10.0F, 6.0F};
    style.ItemSpacing = ImVec2{10.0F, 8.0F};
    style.ItemInnerSpacing = ImVec2{8.0F, 6.0F};
    style.WindowBorderSize = 0.0F;
    style.ChildBorderSize = 0.0F;
    style.FrameBorderSize = 0.0F;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4{0.07F, 0.09F, 0.11F, 1.0F};
    colors[ImGuiCol_ChildBg] = ImVec4{0.10F, 0.12F, 0.15F, 1.0F};
    colors[ImGuiCol_PopupBg] = ImVec4{0.10F, 0.12F, 0.15F, 1.0F};
    colors[ImGuiCol_Border] = ImVec4{0.18F, 0.22F, 0.26F, 0.65F};
    colors[ImGuiCol_FrameBg] = ImVec4{0.14F, 0.17F, 0.21F, 1.0F};
    colors[ImGuiCol_FrameBgHovered] = ImVec4{0.18F, 0.25F, 0.31F, 1.0F};
    colors[ImGuiCol_FrameBgActive] = ImVec4{0.22F, 0.32F, 0.39F, 1.0F};
    colors[ImGuiCol_TitleBg] = ImVec4{0.07F, 0.09F, 0.11F, 1.0F};
    colors[ImGuiCol_TitleBgActive] = ImVec4{0.07F, 0.09F, 0.11F, 1.0F};
    colors[ImGuiCol_Button] = ImVec4{0.14F, 0.45F, 0.39F, 1.0F};
    colors[ImGuiCol_ButtonHovered] = ImVec4{0.18F, 0.57F, 0.49F, 1.0F};
    colors[ImGuiCol_ButtonActive] = ImVec4{0.11F, 0.35F, 0.31F, 1.0F};
    colors[ImGuiCol_Header] = ImVec4{0.14F, 0.20F, 0.25F, 1.0F};
    colors[ImGuiCol_HeaderHovered] = ImVec4{0.18F, 0.27F, 0.33F, 1.0F};
    colors[ImGuiCol_HeaderActive] = ImVec4{0.21F, 0.31F, 0.38F, 1.0F};
    colors[ImGuiCol_CheckMark] = ImVec4{0.42F, 0.82F, 0.73F, 1.0F};
    colors[ImGuiCol_SliderGrab] = ImVec4{0.42F, 0.82F, 0.73F, 1.0F};
    colors[ImGuiCol_SliderGrabActive] = ImVec4{0.58F, 0.90F, 0.83F, 1.0F};
    colors[ImGuiCol_Separator] = ImVec4{0.21F, 0.26F, 0.31F, 1.0F};
}

/// Draws a full-width widget and reports whether its value changed.
template <typename Widget> bool full_width(Widget&& widget) {
    ImGui::SetNextItemWidth(-1.0F);
    return widget();
}

bool labeled_slider(const char* label, float* value, float min, float max, const char* format) {
    ImGui::TextUnformatted(label);
    return full_width([&] {
        return ImGui::SliderFloat((std::string{"##"} + label).c_str(), value, min, max, format,
                                  ImGuiSliderFlags_AlwaysClamp);
    });
}

bool labeled_input(const char* label, float* value) {
    ImGui::TextUnformatted(label);
    return full_width([&] {
        return ImGui::InputFloat((std::string{"##"} + label).c_str(), value, 0.0F, 0.0F, "%.4f");
    });
}

bool labeled_combo(const char* label, int* value, const char* items) {
    ImGui::TextUnformatted(label);
    return full_width(
        [&] { return ImGui::Combo((std::string{"##"} + label).c_str(), value, items); });
}

void draw_config_panel(AppState& app, const ImVec2& size) {
    lab::Settings& s = app.settings;
    bool changed = false;

    if (ImGui::BeginChild("ControlPanel", size, ImGuiChildFlags_Border)) {
        ImGui::TextUnformatted("PID++ Control Lab");
        ImGui::TextDisabled("Tune the controller, run the plant, inspect the response.");

        ImGui::SeparatorText("Plant");
        int plant = s.plant;
        if (labeled_combo("Model", &plant, lab::plant_combo_items()) && plant != s.plant) {
            s = lab::defaults_for(static_cast<lab::PlantModel>(plant));
            app.autotune_message.clear();
            changed = true;
        }
        changed |=
            ImGui::Checkbox("Model feedforward (gravity / hover / kV)", &s.model_feedforward);

        ImGui::SeparatorText("PID gains");
        changed |= labeled_slider("Kp", &s.kp, 0.0F, 200.0F, "%.4f");
        changed |= labeled_slider("Ki", &s.ki, 0.0F, 200.0F, "%.4f");
        changed |= labeled_slider("Kd", &s.kd, 0.0F, 30.0F, "%.4f");
        changed |=
            labeled_slider("Derivative filter tau", &s.derivative_filter_tau, 0.0F, 0.5F, "%.4f s");

        ImGui::SeparatorText("Autotune (relay experiment)");
        full_width([&] { return ImGui::Combo("##Rule", &s.tuning_rule, tuning_rule_items); });
        if (ImGui::Button("Autotune gains", ImVec2{-1.0F, 0.0F})) {
            const auto outcome = lab::autotune(s);
            char buffer[160]{};
            if (outcome.success) {
                s.kp = static_cast<float>(outcome.gains.kp);
                s.ki = static_cast<float>(outcome.gains.ki);
                s.kd = static_cast<float>(outcome.gains.kd);
                std::snprintf(buffer, sizeof(buffer), "Ku = %.4g, Tu = %.4g s",
                              outcome.result.ultimate_gain, outcome.result.ultimate_period);
                app.autotune_message = buffer;
                changed = true;
            } else {
                app.autotune_message = outcome.message;
            }
        }
        if (!app.autotune_message.empty()) {
            ImGui::TextWrapped("%s", app.autotune_message.c_str());
        }

        ImGui::SeparatorText("Limits");
        changed |= labeled_input("Output min", &s.output_min);
        changed |= labeled_input("Output max", &s.output_max);
        changed |= labeled_input("Integral min (error*s)", &s.integral_min);
        changed |= labeled_input("Integral max (error*s)", &s.integral_max);
        changed |= labeled_input("Output rate limit (units/s, 0 = off)", &s.output_rate_limit);
        changed |= labeled_input("Error deadband", &s.error_deadband);

        ImGui::SeparatorText("Controller policy");
        changed |= labeled_combo("Anti-windup", &s.anti_windup, anti_windup_items);
        changed |= labeled_combo("Derivative", &s.derivative_mode, derivative_items);
        changed |=
            labeled_slider("Back-calculation gain", &s.back_calculation_gain, 0.0F, 10.0F, "%.3f");

        ImGui::SeparatorText("Scenario");
        changed |= labeled_input("Setpoint", &s.setpoint);
        changed |= labeled_input("Initial value", &s.initial_measurement);
        changed |= labeled_slider("Time step", &s.dt_ms, 0.1F, 20.0F, "%.2f ms");
        changed |= labeled_slider("Duration", &s.duration_s, 0.25F, 60.0F, "%.2f s");
        changed |= labeled_slider("Measurement noise", &s.noise_amplitude, 0.0F, 1.0F, "+/- %.4f");
        changed |= labeled_slider("Sensor lag", &s.sensor_lag_ms, 0.0F, 100.0F, "%.1f ms");
        changed |= labeled_slider("Dead time", &s.dead_time_ms, 0.0F, 500.0F, "%.0f ms");
        changed |= labeled_input("Disturbance time (s)", &s.disturbance_time);
        changed |= labeled_input("Disturbance (actuator units)", &s.disturbance);

        ImGui::Spacing();
        ImGui::Checkbox("Re-run on every change", &app.auto_run);
        if (ImGui::Button("Run simulation", ImVec2{-1.0F, 36.0F}) || (changed && app.auto_run)) {
            lab::run_simulation(s, app.trace);
        }
        if (ImGui::Button("Export CSV", ImVec2{-1.0F, 0.0F})) {
            app.trace.status = lab::export_csv(app.trace, "pidpp_control_lab.csv")
                                   ? "Exported pidpp_control_lab.csv"
                                   : "CSV export failed";
        }
        ImGui::Spacing();
        ImGui::TextWrapped("%s", app.trace.status.c_str());
    }
    ImGui::EndChild();
}

void draw_metric(const char* label, const char* value) {
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", label);
    ImGui::TextUnformatted(value);
}

void draw_metrics_panel(const AppState& app, const ImVec2& size) {
    if (ImGui::BeginChild("ResultsPanel", size, ImGuiChildFlags_Border)) {
        const auto& m = app.trace.metrics;
        ImGui::Text("Results: %s",
                    lab::plant_name(static_cast<lab::PlantModel>(app.settings.plant)));
        if (ImGui::BeginTable("Metrics", 4, ImGuiTableFlags_SizingStretchSame)) {
            char buffer[64]{};
            std::snprintf(buffer, sizeof(buffer), "%.4f", m.final_value);
            draw_metric("Final value", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.4f", m.peak_value);
            draw_metric("Peak value", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.2f %%", m.overshoot_percent);
            draw_metric("Overshoot", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.4f", m.steady_state_error);
            draw_metric("Steady-state error", buffer);
            if (m.rose) {
                std::snprintf(buffer, sizeof(buffer), "%.4f s", m.rise_time);
            } else {
                std::snprintf(buffer, sizeof(buffer), "did not reach 90 %%");
            }
            draw_metric("Rise time (10-90 %)", buffer);
            if (m.settled) {
                std::snprintf(buffer, sizeof(buffer), "%.4f s", m.settling_time);
            } else {
                std::snprintf(buffer, sizeof(buffer), "not settled");
            }
            draw_metric("Settling (2 %)", buffer);
            std::snprintf(buffer, sizeof(buffer), "%.4f", m.integral_absolute_error);
            draw_metric("IAE", buffer);
            std::snprintf(buffer, sizeof(buffer), "%zu", app.trace.time.size());
            draw_metric("Samples", buffer);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
}

void draw_plot_panel(const AppState& app, const ImVec2& size) {
    if (ImGui::BeginChild("PlotsPanel", size, ImGuiChildFlags_Border)) {
        const auto& trace = app.trace;
        if (trace.time.empty()) {
            ImGui::TextUnformatted("Run a simulation to display plots.");
        } else {
            const int count = static_cast<int>(trace.time.size());
            const float height = ImGui::GetContentRegionAvail().y;
            const float row = std::max(150.0F, (height - 20.0F) / 3.0F);

            if (ImPlot::BeginPlot("Response", ImVec2{-1.0F, row})) {
                ImPlot::SetupAxes("Time [s]", "Value", ImPlotAxisFlags_AutoFit,
                                  ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("Setpoint", trace.time.data(), trace.setpoint.data(), count);
                ImPlot::PlotLine("Measurement", trace.time.data(), trace.measurement.data(), count);
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("Actuator command", ImVec2{-1.0F, row})) {
                ImPlot::SetupAxes("Time [s]", "Command", ImPlotAxisFlags_AutoFit,
                                  ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("Command", trace.time.data(), trace.command.data(), count);
                ImPlot::EndPlot();
            }
            if (ImPlot::BeginPlot("Controller terms", ImVec2{-1.0F, row})) {
                ImPlot::SetupAxes("Time [s]", "Contribution", ImPlotAxisFlags_AutoFit,
                                  ImPlotAxisFlags_AutoFit);
                ImPlot::PlotLine("P", trace.time.data(), trace.p_term.data(), count);
                ImPlot::PlotLine("I", trace.time.data(), trace.i_term.data(), count);
                ImPlot::PlotLine("D", trace.time.data(), trace.d_term.data(), count);
                ImPlot::PlotLine("FF", trace.time.data(), trace.ff_term.data(), count);
                ImPlot::EndPlot();
            }
        }
    }
    ImGui::EndChild();
}

void draw_workspace(AppState& app) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration |
                                              ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                              ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("PIDPP Workspace", nullptr, window_flags)) {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const float gap = 16.0F;
        const float left_width = std::clamp(available.x * 0.30F, 300.0F, 420.0F);
        const float right_width = available.x - left_width - gap;
        const float results_height = 132.0F;

        draw_config_panel(app, ImVec2{left_width, available.y});
        ImGui::SameLine(0.0F, gap);
        if (ImGui::BeginChild("RightColumn", ImVec2{right_width, available.y},
                              ImGuiChildFlags_None)) {
            draw_metrics_panel(app, ImVec2{right_width, results_height});
            draw_plot_panel(app, ImVec2{right_width, available.y - results_height - gap});
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

/// Runs every preset (and an autotune on the stable process plants) without
/// opening a window. Returns a process exit code.
int run_headless() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(lab::PlantModel::Count); ++i) {
        const auto model = static_cast<lab::PlantModel>(i);
        lab::Trace trace;
        lab::run_simulation(lab::defaults_for(model), trace);
        const auto& m = trace.metrics;
        std::printf("%-28s valid=%d final=%9.4f overshoot=%7.2f%% settled=%d settling=%7.3f s\n",
                    lab::plant_name(model), trace.valid ? 1 : 0, m.final_value, m.overshoot_percent,
                    m.settled ? 1 : 0, m.settling_time);
        ok &= trace.valid;
    }
    for (const auto model : {lab::PlantModel::FirstOrder, lab::PlantModel::DcMotorVelocity}) {
        const auto outcome = lab::autotune(lab::defaults_for(model));
        std::printf("autotune %-19s %s (Ku=%.4g Tu=%.4g)\n", lab::plant_name(model),
                    outcome.message.c_str(), outcome.result.ultimate_gain,
                    outcome.result.ultimate_period);
    }
    return ok ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string_view{argv[i]} == "--headless") {
            return run_headless();
        }
    }

    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() == 0) {
        return 1;
    }

    constexpr const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1440, 900, "PID++ Control Lab", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;

    ImGui::StyleColorsDark();
    apply_app_style();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    AppState app{};
    lab::run_simulation(app.settings, app.trace);

    while (glfwWindowShouldClose(window) == 0) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_workspace(app);

        ImGui::Render();

        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.07F, 0.08F, 0.09F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
