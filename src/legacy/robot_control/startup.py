"""Narrow startup classification shared by the service and its console."""

WAITING_FOR_ARM = 75
NO_MOTOR_FEEDBACK = "None of the motors are initialized. Please check the connection or power of the arm."


def waiting_for_arm(error, *, service_started):
    # Do not convert runtime disconnects, partial motor faults or config errors to retries.
    return not service_started and isinstance(error, RuntimeError) and str(error) == NO_MOTOR_FEEDBACK
