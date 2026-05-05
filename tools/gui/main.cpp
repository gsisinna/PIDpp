#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <pidpp/pidpp.hpp>

namespace {

enum class PlantModel {
    FirstOrder,
    DcMotor,
    JointPosition,
};

struct GuiSettings {
    float kp{1.2F};
    float ki{0.4F};
    float kd{0.05F};
    float output_min{-12.0F};
    float output_max{12.0F};
    float integral_min{-10.0F};
    float integral_max{10.0F};
    float derivative_filter_tau{0.02F};
    float back_calculation_gain{1.0F};
    float setpoint{10.0F};
    float initial_measurement{0.0F};
    float dt_ms{1.0F};
    float duration_s{4.0F};
    int anti_windup{static_cast<int>(pidpp::AntiWindupStrategy::ConditionalIntegration)};
    int derivative_mode{static_cast<int>(pidpp::DerivativeMode::OnMeasurement)};
    int plant_model{static_cast<int>(PlantModel::FirstOrder)};
};

struct SimulationData {
    std::vector<double> time{};
    std::vector<double> setpoint{};
    std::vector<double> measurement{};
    std::vector<double> command{};
    pidpp::StepResponseMetrics<double> metrics{};
    std::string status{"Ready"};
};

pidpp::PidConfig<double> make_config(const GuiSettings& settings) {
    pidpp::PidConfig<double> config{};
    config.kp = static_cast<double>(settings.kp);
    config.ki = static_cast<double>(settings.ki);
    config.kd = static_cast<double>(settings.kd);
    config.output_min = static_cast<double>(settings.output_min);
    config.output_max = static_cast<double>(settings.output_max);
    config.integral_min = static_cast<double>(settings.integral_min);
    config.integral_max = static_cast<double>(settings.integral_max);
    config.anti_windup = static_cast<pidpp::AntiWindupStrategy>(settings.anti_windup);
    config.derivative_mode = static_cast<pidpp::DerivativeMode>(settings.derivative_mode);
    config.derivative_filter_tau = static_cast<double>(settings.derivative_filter_tau);
    config.back_calculation_gain = static_cast<double>(settings.back_calculation_gain);
    return config;
}

const char* plant_name(PlantModel model) {
    switch (model) {
    case PlantModel::FirstOrder:
        return "First-order plant";
    case PlantModel::DcMotor:
        return "DC motor velocity";
    case PlantModel::JointPosition:
        return "Joint position";
    }
    return "Unknown";
}

void reserve_samples(SimulationData& data, std::size_t count) {
    data.time.clear();
    data.setpoint.clear();
    data.measurement.clear();
    data.command.clear();
    data.time.reserve(count);
    data.setpoint.reserve(count);
    data.measurement.reserve(count);
    data.command.reserve(count);
}

void push_sample(SimulationData& data, double t, double setpoint, double measurement,
                 double command) {
    data.time.push_back(t);
    data.setpoint.push_back(setpoint);
    data.measurement.push_back(measurement);
    data.command.push_back(command);
}

void run_first_order(const GuiSettings& settings, pidpp::PidController<double>& controller,
                     SimulationData& data, double dt, int steps) {
    pidpp::FirstOrderPlant<double> plant{1.0, 0.35,
                                         static_cast<double>(settings.initial_measurement)};
    double command = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double measurement = plant.update(command, dt);
        command = controller.update(static_cast<double>(settings.setpoint), measurement, dt);
        push_sample(data, t, static_cast<double>(settings.setpoint), measurement, command);
    }
}

void run_motor(const GuiSettings& settings, pidpp::PidController<double>& controller,
               SimulationData& data, double dt, int steps) {
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};
    double voltage = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double speed = motor.update(voltage, 0.0, dt);
        voltage = controller.update(static_cast<double>(settings.setpoint), speed, dt);
        push_sample(data, t, static_cast<double>(settings.setpoint), speed, voltage);
    }
}

void run_joint(const GuiSettings& settings, pidpp::PidController<double>& controller,
               SimulationData& data, double dt, int steps) {
    pidpp::MassSpringDamper<double> joint{1.2, 1.4, 0.0};
    joint.reset(static_cast<double>(settings.initial_measurement), 0.0);
    double torque = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double position = joint.update(torque, dt);
        torque = controller.update(static_cast<double>(settings.setpoint), position, dt);
        push_sample(data, t, static_cast<double>(settings.setpoint), position, torque);
    }
}

void run_simulation(const GuiSettings& settings, SimulationData& data) {
    const auto config = make_config(settings);
    const auto validation = config.validate();
    if (!validation) {
        data.status = std::string{"Invalid configuration: "} + validation.message();
        return;
    }

    const double dt = static_cast<double>(settings.dt_ms) / 1000.0;
    if (!pidpp::validate_time_step(dt)) {
        data.status = "Invalid simulation step";
        return;
    }

    const int steps =
        static_cast<int>(std::max(1.0, std::floor(static_cast<double>(settings.duration_s) / dt)));
    reserve_samples(data, static_cast<std::size_t>(steps));

    pidpp::PidController<double> controller{config};
    const auto model = static_cast<PlantModel>(settings.plant_model);
    switch (model) {
    case PlantModel::FirstOrder:
        run_first_order(settings, controller, data, dt, steps);
        break;
    case PlantModel::DcMotor:
        run_motor(settings, controller, data, dt, steps);
        break;
    case PlantModel::JointPosition:
        run_joint(settings, controller, data, dt, steps);
        break;
    }

    data.metrics = pidpp::analyze_step_response<double>(data.time, data.measurement,
                                                        static_cast<double>(settings.setpoint));
    data.status = "Simulation complete";
}

void apply_app_style() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 14.0F;
    style.ChildRounding = 12.0F;
    style.FrameRounding = 10.0F;
    style.PopupRounding = 10.0F;
    style.GrabRounding = 10.0F;
    style.ScrollbarRounding = 10.0F;
    style.TabRounding = 10.0F;
    style.WindowPadding = ImVec2{16.0F, 16.0F};
    style.FramePadding = ImVec2{12.0F, 8.0F};
    style.ItemSpacing = ImVec2{10.0F, 10.0F};
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

void set_full_width() {
    ImGui::SetNextItemWidth(-1.0F);
}

void draw_metric_card(const char* label, const char* value) {
    ImGui::BeginChild(label, ImVec2{0.0F, 58.0F}, ImGuiChildFlags_Border);
    ImGui::TextUnformatted(label);
    ImGui::Spacing();
    ImGui::TextDisabled("%s", value);
    ImGui::EndChild();
}

void draw_config_panel(GuiSettings& settings, SimulationData& data, const ImVec2& size) {
    if (ImGui::BeginChild("ControlPanel", size, ImGuiChildFlags_Border)) {
        ImGui::TextUnformatted("Control Lab");
        ImGui::TextDisabled("Tune the controller, run the plant, inspect the response.");
        ImGui::Spacing();

        ImGui::SeparatorText("Plant");
        set_full_width();
        ImGui::Combo("##Plant", &settings.plant_model,
                     "First-order plant\0DC motor velocity\0Joint position\0");

        ImGui::SeparatorText("PID Gains");
        set_full_width();
        ImGui::SliderFloat("Kp", &settings.kp, 0.0F, 100.0F, "%.4f");
        set_full_width();
        ImGui::SliderFloat("Ki", &settings.ki, 0.0F, 50.0F, "%.4f");
        set_full_width();
        ImGui::SliderFloat("Kd", &settings.kd, 0.0F, 20.0F, "%.4f");
        set_full_width();
        ImGui::SliderFloat("Derivative tau", &settings.derivative_filter_tau, 0.0F, 1.0F, "%.4f s");

        ImGui::SeparatorText("Limits");
        set_full_width();
        ImGui::InputFloat("Output min", &settings.output_min);
        set_full_width();
        ImGui::InputFloat("Output max", &settings.output_max);
        set_full_width();
        ImGui::InputFloat("Integral min", &settings.integral_min);
        set_full_width();
        ImGui::InputFloat("Integral max", &settings.integral_max);

        ImGui::SeparatorText("Controller Policy");
        set_full_width();
        ImGui::Combo("Anti-windup", &settings.anti_windup,
                     "None\0Clamping\0Conditional integration\0Back calculation\0");
        set_full_width();
        ImGui::Combo("Derivative", &settings.derivative_mode, "On error\0On measurement\0");
        set_full_width();
        ImGui::SliderFloat("Back calc gain", &settings.back_calculation_gain, 0.0F, 10.0F, "%.4f");

        ImGui::SeparatorText("Simulation");
        set_full_width();
        ImGui::InputFloat("Setpoint", &settings.setpoint);
        set_full_width();
        ImGui::InputFloat("Initial value", &settings.initial_measurement);
        set_full_width();
        ImGui::SliderFloat("dt", &settings.dt_ms, 0.1F, 20.0F, "%.3f ms");
        set_full_width();
        ImGui::SliderFloat("Duration", &settings.duration_s, 0.25F, 20.0F, "%.2f s");

        ImGui::Spacing();
        if (ImGui::Button("Run Simulation", ImVec2{-1.0F, 42.0F})) {
            run_simulation(settings, data);
        }

        ImGui::Spacing();
        ImGui::TextUnformatted("Status");
        ImGui::TextWrapped("%s", data.status.c_str());
    }
    ImGui::EndChild();
}

void draw_metrics_panel(const GuiSettings& settings, const SimulationData& data,
                        const ImVec2& size) {
    if (ImGui::BeginChild("ResultsPanel", size, ImGuiChildFlags_Border)) {
        ImGui::TextUnformatted("Results");
        ImGui::TextDisabled("%s", plant_name(static_cast<PlantModel>(settings.plant_model)));
        ImGui::Spacing();

        char buffer[64]{};
        std::snprintf(buffer, sizeof(buffer), "%zu samples", data.time.size());
        draw_metric_card("Samples", buffer);

        std::snprintf(buffer, sizeof(buffer), "%.4f", data.metrics.final_value);
        draw_metric_card("Final value", buffer);

        std::snprintf(buffer, sizeof(buffer), "%.4f", data.metrics.peak_value);
        draw_metric_card("Peak value", buffer);

        std::snprintf(buffer, sizeof(buffer), "%.2f %%", data.metrics.overshoot_percent);
        draw_metric_card("Overshoot", buffer);

        std::snprintf(buffer, sizeof(buffer), "%.4f s", data.metrics.rise_time);
        draw_metric_card("Rise time", buffer);

        if (data.metrics.settled) {
            std::snprintf(buffer, sizeof(buffer), "%.4f s", data.metrics.settling_time);
        } else {
            std::snprintf(buffer, sizeof(buffer), "%s", "not settled");
        }
        draw_metric_card("Settling", buffer);
    }
    ImGui::EndChild();
}

void draw_plot_panel(const SimulationData& data, const ImVec2& size) {
    if (ImGui::BeginChild("PlotsPanel", size, ImGuiChildFlags_Border)) {
        ImGui::TextUnformatted("Plots");
        ImGui::TextDisabled("Response and actuator behavior across the simulation horizon.");
        ImGui::Spacing();

        if (data.time.empty()) {
            ImGui::TextUnformatted("Run a simulation to display plots.");
        } else {
            const int count = static_cast<int>(data.time.size());
            const float available_height = ImGui::GetContentRegionAvail().y;
            const float response_height = std::max(240.0F, available_height * 0.6F);
            const float command_height =
                std::max(180.0F, available_height - response_height - 18.0F);

            if (ImPlot::BeginPlot("Response", ImVec2{-1.0F, response_height})) {
                ImPlot::SetupAxes("Time [s]", "Value");
                ImPlot::PlotLine("Setpoint", data.time.data(), data.setpoint.data(), count);
                ImPlot::PlotLine("Measurement", data.time.data(), data.measurement.data(), count);
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Controller command", ImVec2{-1.0F, command_height})) {
                ImPlot::SetupAxes("Time [s]", "Command");
                ImPlot::PlotLine("Command", data.time.data(), data.command.data(), count);
                ImPlot::EndPlot();
            }
        }
    }
    ImGui::End();
}

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

void draw_main_workspace(GuiSettings& settings, SimulationData& data) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    constexpr ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration |
                                              ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                              ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("PIDPP Workspace", nullptr, window_flags)) {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const float left_width = std::clamp(available.x * 0.30F, 300.0F, 420.0F);
        const float gap = 16.0F;
        const float right_width = available.x - left_width - gap;
        const float results_height = std::clamp(available.y * 0.32F, 220.0F, 340.0F);

        draw_config_panel(settings, data, ImVec2{left_width, available.y});

        ImGui::SameLine(0.0F, gap);
        if (ImGui::BeginChild("RightColumn", ImVec2{right_width, available.y},
                              ImGuiChildFlags_None)) {
            draw_metrics_panel(settings, data, ImVec2{right_width, results_height});
            draw_plot_panel(data, ImVec2{right_width, available.y - results_height - gap});
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace

int main() {
    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() == 0) {
        return 1;
    }

    constexpr const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1280, 800, "PIDPP Control Lab", nullptr, nullptr);
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

    ImGui::StyleColorsDark();
    apply_app_style();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    GuiSettings settings{};
    SimulationData data{};
    run_simulation(settings, data);

    while (glfwWindowShouldClose(window) == 0) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_main_workspace(settings, data);

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
