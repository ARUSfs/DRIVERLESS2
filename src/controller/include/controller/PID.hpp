/**
 * @class PID
 * @brief PID class 
 * 
 * Controller pp
 */
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

/**
 * @class PID
 * @brief PID class 
 * 
 *  velocity control
 */

class PID {
public:
    // Constructor solo inicializa el nodo
    PID(rclcpp::Node::SharedPtr node)
        : node_(node) {
        initialize();
    }

    // Método para calcular el control de aceleración basado en la velocidad actual
    double compute_control() {
        auto current_time = node_->now();
        auto delta_time = (current_time - previous_time).seconds();
        previous_time = current_time;

        // Calcula el error de velocidad
        const float v_error = TARGET_SPEED - velocity;
        integral += v_error * delta_time;
        const float derivative = (v_error - previous_error) / delta_time;
        previous_error = v_error;

        // Control de alimentación (feed-forward)
        const float feed_forward = (TARGET_SPEED - prev_target) / delta_time;
        prev_target = TARGET_SPEED;

        // Cálculo del control del acelerador utilizando el PID
        const float accelerator_control = (v_error * KP) + (KI * integral) + (KD * derivative) + feed_forward;

        // Devolver el valor del control del acelerador
        return accelerator_control / 230; // Normaliza el control
    }

    // Métodos para establecer los valores de velocidad y objetivo
    void set_speed(float current_velocity) {
        velocity = current_velocity;
    }

    void set_target_speed(float target_speed) {
        TARGET_SPEED = target_speed;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time previous_time;

    // Parámetros del controlador PID inicializados en el lugar de declaración
    float TARGET_SPEED = 0.0;
    float KP = 0.0, KD = 0.0, KI = 0.0;
    float previous_error = 0.0;
    float integral = 0.0;
    float velocity = 0.0;
    float prev_target = 0.0;

    // Inicializa los parámetros desde el nodo
    void initialize() {
        node_->declare_parameter<float>("TARGET_SPEED", 0.0);
        node_->declare_parameter<float>("KP", 0.0);
        node_->declare_parameter<float>("KD", 0.0);
        node_->declare_parameter<float>("KI", 0.0);

        node_->get_parameter("TARGET_SPEED", TARGET_SPEED);
        node_->get_parameter("KP", KP);
        node_->get_parameter("KD", KD);
        node_->get_parameter("KI", KI);

        previous_time = node_->now();
    }
};






