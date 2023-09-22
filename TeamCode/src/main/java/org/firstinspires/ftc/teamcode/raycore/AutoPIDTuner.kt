package org.firstinspires.ftc.teamcode.raycore

data class PIDParameters(var kp: Double, var ki: Double, var kd: Double)

data class SystemResponse(val overshoot: Double, val settlingTime: Double)

fun calculateSystemResponse(errorValues: List<Double>): SystemResponse {
    // Calculate overshoot and settling time from error values
    // You can use various techniques to extract these values from the error data
    // For simplicity, let's assume you have a method to calculate them
    var

    val overshoot = errorValues.forEach();// Calculate overshoot
    val settlingTime = // Calculate settling time

            return SystemResponse(overshoot, settlingTime)
}

fun autoTunePID(currentParameters: PIDParameters, errorValues: List<Double>): PIDParameters {
    val initialParameters = currentParameters.copy()  // Store initial parameters
    val initialResponse = calculateSystemResponse(errorValues)

    // Modify PID parameters and observe system response
    // You can use Ziegler-Nichols or other methods here to adjust the parameters

    // For simplicity, let's just increase P, I, and D by a constant factor
    val updatedParameters = currentParameters.copy(
            kp = currentParameters.kp * 1.1,
            ki = currentParameters.ki * 1.2,
            kd = currentParameters.kd * 0.9
    )

    val updatedResponse = calculateSystemResponse(errorValues)

    // Compare the responses and decide whether to keep the updated parameters
    val initialScore = calculateScore(initialResponse)
    val updatedScore = calculateScore(updatedResponse)

    return if (updatedScore < initialScore) {
        updatedParameters
    } else {
        initialParameters
    }
}

fun calculateScore(response: SystemResponse): Double {
    // You can define a scoring function based on overshoot and settling time
    // Lower score indicates a better response
    return response.overshoot * response.settlingTime
}

fun main() {
    val initialParameters = PIDParameters(kp = 1.0, ki = 0.5, kd = 0.2)
    val errorValues = listOf(10.0, -5.0, 8.0, -3.0, 6.0)  // Example error values

    val tunedParameters = autoTunePID(initialParameters, errorValues)
    println("Tuned PID Parameters: Kp=${tunedParameters.kp}, Ki=${tunedParameters.ki}, Kd=${tunedParameters.kd}")
}
