from ctre import CANifier, CANifierStatusFrame, ErrorCode


class CANifiedPWMEncoder:
    def __init__(
        self,
        canifier_id: int,
        pwm_channel: int,
        offset: float,
        ratio: float,
        inverted: bool,
        update_rate: int = 5,  # milliseconds
    ) -> None:
        print(f"Initializing CANifiedPWMEncoder {canifier_id}")
        self.canifier_id = canifier_id
        self.offset = offset
        self.ratio = ratio
        self.inverted = inverted
        self.update_rate = update_rate

        self.channels = [
            CANifier.PWMChannel.PWMChannel0,
            CANifier.PWMChannel.PWMChannel1,
            CANifier.PWMChannel.PWMChannel2,
            CANifier.PWMChannel.PWMChannel3,
        ]
        self.status_frames = [
            CANifierStatusFrame.Status_3_PwmInputs0,
            CANifierStatusFrame.Status_4_PwmInputs1,
            CANifierStatusFrame.Status_5_PwmInputs2,
            CANifierStatusFrame.Status_6_PwmInputs3,
        ]
        self.canifier = CANifier(self.canifier_id)
        self.channel = self.channels[pwm_channel]

        for status_frame in self.status_frames:
            self.canifier.setStatusFramePeriod(status_frame, self.update_rate)

        self.prev_position = 0.0
        print(f"CANifiedPWMEncoder {self.canifier_id} is ready")

    def get_position(self) -> float:
        error_code, data = self.canifier.getPWMInput(self.channel)
        if error_code != ErrorCode.OK:
            print(
                f"CANified PWM Channel {self.channel.name} returned an error code: {error_code}"
            )
            return self.prev_position
        duty_cycle, period = data
        if period == 0.0:
            print(f"CANified PWM Channel {self.channel.name} is disconnected!")
            return self.prev_position
        self.prev_position = (-1.0 if self.inverted else 1.0) * (
            self.ratio * duty_cycle / period
        ) - self.offset
        return self.prev_position
