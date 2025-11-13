#include "myCtrl.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <random>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyController::MyController(const LayoutPosition *position, const string &name) : ControlLaw(position->getLayout(),name,4)
{
    first_update = true;
    delta_t = 0.0f;
    initial_time = 0.0f;
    wind_data_loaded = false;
    wind_active = false;
    wind_index = 0;
    wind_velocity = Vector3Df(0.0f, 0.0f, 0.0f);
    wind_start_time = 0.0f;

    // Input matrix
    input = new Matrix(this, 4, 5, floatType, name);

    // Matrix descriptor for logging. It should be always a nx1 matrix. 
    MatrixDescriptor *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "x_error");
    log_labels->SetElementName(1, 0, "y_error");
    log_labels->SetElementName(2, 0, "yaw_error");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    MatrixDescriptor *wind_labels = new MatrixDescriptor(3, 1);
    wind_labels->SetElementName(0, 0, "wind_vx");
    wind_labels->SetElementName(1, 0, "wind_vy");
    wind_labels->SetElementName(2, 0, "wind_vz");
    wind_log = new Matrix(this, wind_labels, floatType, name + string("_wind"));
    delete wind_labels;

    // GUI for custom PID
    GroupBox *gui_customPID = new GroupBox(position, name);
    GroupBox *general_parameters = new GroupBox(gui_customPID->NewRow(), "General parameters");
    deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    mass = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Mass [kg]", 0, 10, 0.01, 4, 0.436);
    k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Motor constant", 0, 50, 0.01, 4, 29.5870);
    sat_pos = new DoubleSpinBox(general_parameters->NewRow(), "Saturation pos", 0, 10, 0.01, 3);
    sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation att", 0, 10, 0.01, 3);
    sat_thrust = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Saturation thrust", 0, 10, 0.01, 3);

    // Custom cartesian position controller
    GroupBox *custom_position = new GroupBox(gui_customPID->NewRow(), "Custom position controller");
    Kp_pos = new Vector3DSpinBox(custom_position->NewRow(), "Kp_pos", 0, 100, 0.1, 3);
    Kd_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Kd_pos", 0, 100, 0.1, 3);
    Ki_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Ki_pos", 0, 100, 0.1, 3);

    // Custom attitude controller
    GroupBox *custom_attitude = new GroupBox(gui_customPID->NewRow(), "Custom attitude controller");
    Kp_att = new Vector3DSpinBox(custom_attitude->NewRow(), "Kp_att", 0, 100, 0.1, 3);
    Kd_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Kd_att", 0, 100, 0.1, 3);
    Ki_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Ki_att", 0, 100, 0.1, 3);

    GroupBox *wind_controls = new GroupBox(gui_customPID->NewRow(), "Wind perturbations");
    start_wind_button = new PushButton(wind_controls->NewRow(), "Start wind");
    stop_wind_button = new PushButton(wind_controls->LastRowLastCol(), "Stop wind");

    AddDataToLog(state);
    AddDataToLog(wind_log);

    loadWindSamples();
}

MyController::~MyController()
{
    delete state;
    delete wind_log;
}

void MyController::UpdateFrom(const io_data *data)
{
    float current_time = double(GetTime())/1000000000-initial_time;
    float thrust = 0.0;
    Vector3Df u, tau;

    if(deltaT_custom->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000;
    }
    else
    {
        delta_t = deltaT_custom->Value();
    }
   
    if(first_update)
    {
        initial_time = double(GetTime())/1000000000;
        first_update = false;
        resetWindState();
    }

    if (start_wind_button->Clicked())
    {
        if (wind_data_loaded)
        {
            resetWindState();
            wind_start_time = current_time;
            wind_active = true;
            std::cout << "Wind perturbations started" << std::endl;
        }
        else
        {
            std::cout << "Wind perturbations unavailable: wind data file not loaded" << std::endl;
        }
    }

    if (stop_wind_button->Clicked())
    {
        if (wind_active)
        {
            wind_active = false;
            resetWindState();
            std::cout << "Wind perturbations stopped" << std::endl;
        }
    }

    updateWind(current_time);

    // Obtain state
    input->GetMutex();
    Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    Quaternion q(input->Value(0, 2), input->Value(1, 2), input->Value(2, 2), input->Value(3, 2));
    Vector3Df omega(input->Value(0, 3), input->Value(1, 3), input->Value(2, 3));
    float yaw_ref = input->Value(0, 4);
    input->ReleaseMutex();

    // Get tunning parameters from GUI
    Vector3Df Kp_pos_val(Kp_pos->Value().x, Kp_pos->Value().y, Kp_pos->Value().z);
    Vector3Df Kd_pos_val(Kd_pos->Value().x, Kd_pos->Value().y, Kd_pos->Value().z);
    Vector3Df Ki_pos_val(Ki_pos->Value().x, Ki_pos->Value().y, Ki_pos->Value().z);
    Vector3Df Kp_att_val(Kp_att->Value().x, Kp_att->Value().y, Kp_att->Value().z);
    Vector3Df Kd_att_val(Kd_att->Value().x, Kd_att->Value().y, Kd_att->Value().z);
    Vector3Df Ki_att_val(Ki_att->Value().x, Ki_att->Value().y, Ki_att->Value().z);

    Vector3Df compensated_vel_error(vel_error);
    compensated_vel_error.x -= wind_velocity.x;

    // Cartesian custom controller
    u.x = Kp_pos_val.x*pos_error.x + Kd_pos_val.x*compensated_vel_error.x;
    u.y = Kp_pos_val.y*pos_error.y + Kd_pos_val.y*compensated_vel_error.y;
    u.z = Kp_pos_val.z*pos_error.z + Kd_pos_val.z*compensated_vel_error.z;
    float ctrl_z = u.z; // This is the thrust needed to control the z position before saturation
    u.Saturate(sat_pos->Value());

    // Attitude custom controller
    Euler rpy = q.ToEuler();    
    tau.x = Kp_att_val.x*(rpy.roll + u.y) + Kd_att_val.x*omega.x;
    tau.y = Kp_att_val.y*(rpy.pitch - u.x) + Kd_att_val.y*omega.y;
    tau.z = Kp_att_val.z*(rpy.YawDistanceFrom(yaw_ref)) + Kd_att_val.z*omega.z;
    applyMotorConstant(tau);
    tau.Saturate(sat_att->Value());

    // Compute custom thrust
    float comp_mg = -mass->Value()*g; // This is the thrust needed to counteract gravity. Based on the default PID, it should be -0.397918 in Fl-Air simulator.  
    thrust = comp_mg + ctrl_z; // This is the thrust needed to counteract gravity and control the z position
    applyMotorConstant(thrust);
    if(thrust < -sat_att->Value())
    {
        thrust = -sat_att->Value();
    }
    else if(thrust >= 0)
    {
        thrust = 0; 
    }
    // Debug thrust value
    std::cout << " error_x: " << pos_error.x << " error_y: " << pos_error.y << " error_z: " << pos_error.z;
    if (wind_data_loaded)
    {
        std::cout << " wind_vx: " << wind_velocity.x ;
    }
    std::cout << std::endl;

    // Send controller output
    output->SetValue(0, 0, tau.x);
    output->SetValue(1, 0, tau.y);
    output->SetValue(2, 0, tau.z);
    output->SetValue(3, 0, thrust);
    output->SetDataTime(data->DataTime());

    // Log state (example). 
    // Modify the log_labels matrix in the constructor to add more variables.
    state->GetMutex();
    state->SetValue(0, 0, pos_error.x);
    state->SetValue(1, 0, pos_error.y);
    state->SetValue(2, 0, rpy.YawDistanceFrom(0));
    state->ReleaseMutex();

    wind_log->GetMutex();
    wind_log->SetValue(0, 0, wind_velocity.x);
    wind_log->SetValue(1, 0, wind_velocity.y);
    wind_log->SetValue(2, 0, wind_velocity.z);
    wind_log->ReleaseMutex();

    ProcessUpdate(output);
}

void MyController::Reset(void)
{
    first_update = true;
    wind_active = false;
    resetWindState();
}

void MyController::SetValues(Vector3Df pos_error, Vector3Df vel_error, Quaternion currentQuaternion, Vector3Df omega, float yaw_ref)
{
    // Set the input values for the controller. 
    // This function is called from the main controller to set the input values.
    input->GetMutex();
    input->SetValue(0, 0, pos_error.x);
    input->SetValue(1, 0, pos_error.y);
    input->SetValue(2, 0, pos_error.z);

    input->SetValue(0, 1, vel_error.x);
    input->SetValue(1, 1, vel_error.y);
    input->SetValue(2, 1, vel_error.z);

    input->SetValue(0, 2, currentQuaternion.q0);
    input->SetValue(1, 2, currentQuaternion.q1);
    input->SetValue(2, 2, currentQuaternion.q2);
    input->SetValue(3, 2, currentQuaternion.q3);

    input->SetValue(0, 3, omega.x);
    input->SetValue(1, 3, omega.y);
    input->SetValue(2, 3, omega.z);

    // Set yaw reference
    input->SetValue(0, 4, yaw_ref);
    input->ReleaseMutex();
}

void MyController::applyMotorConstant(Vector3Df &signal)
{
    float motor_constant = k_motor->Value();
    signal.x = signal.x/motor_constant;
    signal.y = signal.y/motor_constant;
    signal.z = signal.z/motor_constant;
}

void MyController::applyMotorConstant(float &signal)
{
    float motor_constant = k_motor->Value();
    signal = signal/motor_constant;
}
void MyController::loadWindSamples()
{
    wind_samples.clear();
    wind_data_source.clear();
    wind_data_loaded = false;

    struct IntervalParams
    {
        float start;
        float end;
        float mean;
        float stddev;
    };

        const std::vector<IntervalParams> intervals = {
        {0.0f, 5.0f, 0.11f, 0.92f},
        {5.0f, 10.0f, -0.24f, 1.37f},
        {10.0f, 18.0f, -0.53f, 1.78f}
    };    

    const float total_duration = 18.0f;
    const float sampling_period = 0.1f;

    std::mt19937 rng(0u);

    for (float time = 0.0f; time <= total_duration + std::numeric_limits<float>::epsilon(); time += sampling_period)
    {
        const IntervalParams *active_interval = &intervals.back();
        for (const auto &interval : intervals)
        {
            if (time >= interval.start && time < interval.end)
            {
                active_interval = &interval;
                break;
            }
        }

        std::normal_distribution<float> distribution(active_interval->mean, active_interval->stddev);
        float gust = distribution(rng);
        
        WindSample sample;
        sample.time = time;
        sample.velocity = Vector3Df(gust, 0.0f, 0.0f);
        wind_samples.push_back(sample);
    }
    
    if (!wind_samples.empty())
    {
        wind_data_loaded = true;
        wind_active = false;
        wind_start_time = 0.0f;
        wind_data_source = "synthetic_gaussian_18s";
        std::cout << "Wind perturbations procedurally generated (" << wind_samples.size() << " samples)" << std::endl;
        resetWindState();
    }
    else
    {
        std::cerr << "MyController: unable to generate wind perturbations." << std::endl;
    }
}

void MyController::resetWindState()
{
    wind_index = 0;
    wind_velocity = Vector3Df(0.0f, 0.0f, 0.0f);
}

void MyController::updateWind(float current_time)
{
    if (!wind_data_loaded || wind_samples.empty() || !wind_active)
    {
        wind_velocity = Vector3Df(0.0f, 0.0f, 0.0f);
        return;
    }

    float elapsed_time = current_time - wind_start_time;
    if (elapsed_time < 0.0f)
    {
        elapsed_time = 0.0f;
    }

    while ((wind_index + 1) < wind_samples.size() && elapsed_time >= wind_samples[wind_index + 1].time)
    {
        wind_index++;
    }

    wind_velocity = interpolateWind(elapsed_time);
}

Vector3Df MyController::interpolateWind(float elapsed_time)
{
    if (wind_samples.empty())
    {
        return Vector3Df(0.0f, 0.0f, 0.0f);
    }

    const WindSample &current_sample = wind_samples[wind_index];

    if (wind_index + 1 >= wind_samples.size())
    {
        return current_sample.velocity;
    }

    const WindSample &next_sample = wind_samples[wind_index + 1];
    float span = next_sample.time - current_sample.time;
    float alpha = 0.0f;

    if (span > std::numeric_limits<float>::epsilon())
    {
        alpha = (elapsed_time - current_sample.time) / span;
        if (alpha < 0.0f)
        {
            alpha = 0.0f;
        }
        else if (alpha > 1.0f)
        {
            alpha = 1.0f;
        }
    }

    Vector3Df interpolated;
    interpolated.x = current_sample.velocity.x + (next_sample.velocity.x - current_sample.velocity.x) * alpha;
    interpolated.y = current_sample.velocity.y + (next_sample.velocity.y - current_sample.velocity.y) * alpha;
    interpolated.z = current_sample.velocity.z + (next_sample.velocity.z - current_sample.velocity.z) * alpha;

    return interpolated;
}