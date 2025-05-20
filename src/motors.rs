use esp_idf_hal::ledc::LedcDriver;
use esp_idf_hal::sys::EspError;

use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread;

use esp_idf_hal::delay::Delay;

#[derive(Clone, Copy, PartialEq)]
pub enum MotorState {
    Stop,
    Left,
    Right,
    Forward,
    Backwards,
}

#[derive(Clone)]
enum MotorDirection {
    Stop,
    Forward,
    Backwards,
}

impl MotorDirection {
    fn invert(&self) -> Self {
        match self {
            MotorDirection::Stop => MotorDirection::Stop,
            MotorDirection::Forward => MotorDirection::Backwards,
            MotorDirection::Backwards => MotorDirection::Forward,
        }
    }
}

#[derive(Clone)]
pub struct MotorConfig {
    left_duty: u32,
    right_duty: u32,
    left_invert: bool,
    right_invert: bool,
}

pub struct Motors {
    config: MotorConfig,
}

impl MotorConfig {
    pub fn new(left_duty: u32, right_duty: u32, left_invert: bool, right_invert: bool) -> Self {
        Self {
            left_duty,
            right_duty,
            left_invert,
            right_invert,
        }
    }
}

impl Motors {
    pub fn new(config: MotorConfig) -> Result<Self, EspError> {
        Ok(Self { config })
    }

    fn driver_set<'d>(
        inp_1: &mut LedcDriver<'d>,
        inp_2: &mut LedcDriver<'d>,
        dir: MotorDirection,
        motor_duty: u32,
        invert: bool,
    ) {
        let mut dir = dir;
        if invert {
            dir = dir.invert();
        }

        match dir {
            MotorDirection::Stop => {
                inp_1.set_duty(0).unwrap();
                inp_2.set_duty(0).unwrap();
            }
            MotorDirection::Forward => {
                inp_1.set_duty(0).unwrap();
                inp_2
                    .set_duty(motor_duty * inp_2.get_max_duty() / 100)
                    .unwrap();
            }
            MotorDirection::Backwards => {
                inp_1
                    .set_duty(motor_duty * inp_1.get_max_duty() / 100)
                    .unwrap();
                inp_2.set_duty(0).unwrap();
            }
        }
    }

    pub fn start(
        &mut self,
        mut motor_left_1: LedcDriver<'static>,
        mut motor_left_2: LedcDriver<'static>,
        mut motor_right_1: LedcDriver<'static>,
        mut motor_right_2: LedcDriver<'static>,
    ) -> Result<Sender<MotorState>, EspError> {
        let config = self.config.clone();
        let mut state = MotorState::Stop;
        let (tx, rx): (Sender<MotorState>, Receiver<MotorState>) = mpsc::channel();

        thread::spawn(move || {
            let delay: Delay = Default::default();
            motor_left_1.set_duty(0).unwrap();
            motor_left_2.set_duty(0).unwrap();
            motor_right_1.set_duty(0).unwrap();
            motor_right_2.set_duty(0).unwrap();
            loop {
                let mut left = MotorDirection::Stop;
                let mut right = MotorDirection::Stop;
                if let Ok(new_state) = rx.try_recv() {
                    state = new_state;
                }

                match state {
                    MotorState::Stop => {
                        // left and right are already stopped
                    }
                    MotorState::Forward => {
                        left = MotorDirection::Forward;
                        right = MotorDirection::Forward;
                    }
                    MotorState::Backwards => {
                        left = MotorDirection::Backwards;
                        right = MotorDirection::Backwards;
                    }
                    MotorState::Left => {
                        left = MotorDirection::Stop;
                        right = MotorDirection::Forward;
                    }
                    MotorState::Right => {
                        left = MotorDirection::Forward;
                        right = MotorDirection::Stop;
                    }
                }

                Motors::driver_set(
                    &mut motor_left_1,
                    &mut motor_left_2,
                    left,
                    config.left_duty,
                    config.left_invert,
                );
                Motors::driver_set(
                    &mut motor_right_1,
                    &mut motor_right_2,
                    right,
                    config.right_duty,
                    config.right_invert,
                );

                delay.delay_ms(10);
            }
        });
        Ok(tx)
    }
}
