#ifndef MYCTRL_H
#define MYCTRL_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>
#include <cstddef>
#include <string>
#include <vector>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
        class Vector3DSpinBox;
        class PushButton;
    }
    namespace filter {
        // If you prefer to use a custom controller class, you can define it here.
        // ...
    }
}

namespace flair {
    namespace filter {
        class MyController : public ControlLaw
        {
            public :
                MyController(const flair::gui::LayoutPosition *position, const std::string &name);
                ~MyController();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(flair::core::Vector3Df pos_error, flair::core::Vector3Df vel_error, flair::core::Quaternion currentQuaternion, flair::core::Vector3Df omega, float yaw_ref);
                void applyMotorConstant(flair::core::Vector3Df &signal);
                void applyMotorConstant(float &signal);

            private :
                struct WindSample
                {
                    float time;
                    flair::core::Vector3Df velocity;
                };

                void loadWindSamples();
                void updateWind(float current_time);
                void resetWindState();
                flair::core::Vector3Df interpolateWind(float current_time);

                float delta_t, initial_time;
                float g = 9.81;
                bool first_update;

                bool wind_data_loaded;
                bool wind_active;
                std::string wind_data_source;
                std::vector<WindSample> wind_samples;
                std::size_t wind_index;
                flair::core::Vector3Df wind_velocity;
                float wind_start_time;

                flair::core::Matrix *state;
                flair::core::Matrix *wind_log;
                flair::gui::Vector3DSpinBox *Kp_pos, *Kd_pos, *Ki_pos, *Kp_att, *Kd_att, *Ki_att;
                flair::gui::DoubleSpinBox *deltaT_custom, *mass, *k_motor, *sat_pos, *sat_att, *sat_thrust;
                flair::gui::PushButton *start_wind_button;
                flair::gui::PushButton *stop_wind_button;
        };
    }
}

#endif // MYCTRL_H