use stm32f4 as pac;

pub struct Leds<'a> {
    gpiod: &'a pac::gpiod::RegisterBlock,
}

impl<'a> Leds<'a> {
    pub fn new(p: &'a pac::Peripherals) -> Self {
        Self {
            gpiod: unsafe { &*p.gpiod.moder().as_ptr().cast::<pac::gpiod::RegisterBlock>() },
        }
    }

    pub fn init(&mut self) {
        unsafe {
            self.gpiod.moder().modify(|_, w| {
                w.moder12().output()
                 .moder13().output()
                 .moder14().output()
                 .moder15().output()
            });
            
            self.gpiod.otyper().modify(|_, w| {
                w.ot12().push_pull()
                 .ot13().push_pull()
                 .ot14().push_pull()
                 .ot15().push_pull()
            });
            
            self.gpiod.ospeedr().modify(|_, w| {
                w.ospeedr12().low_speed()
                 .ospeedr13().low_speed()
                 .ospeedr14().low_speed()
                 .ospeedr15().low_speed()
            });
            
            self.gpiod.bsrr().write(|w| {
                w.br12().set_bit()
                 .br13().set_bit()
                 .br14().set_bit()
                 .br15().set_bit()
            });
        }
    }

    pub fn set(&mut self, led: u8, state: bool) {
        unsafe {
            match (led, state) {
                (0, true)  => { let _ = self.gpiod.bsrr().write(|w| w.bs12().set_bit()); }
                (0, false) => { let _ = self.gpiod.bsrr().write(|w| w.br12().set_bit()); }
                (1, true)  => { let _ = self.gpiod.bsrr().write(|w| w.bs13().set_bit()); }
                (1, false) => { let _ = self.gpiod.bsrr().write(|w| w.br13().set_bit()); }
                (2, true)  => { let _ = self.gpiod.bsrr().write(|w| w.bs14().set_bit()); }
                (2, false) => { let _ = self.gpiod.bsrr().write(|w| w.br14().set_bit()); }
                (3, true)  => { let _ = self.gpiod.bsrr().write(|w| w.bs15().set_bit()); }
                (3, false) => { let _ = self.gpiod.bsrr().write(|w| w.br15().set_bit()); }
                _ => {}
            }
        }
    }

    pub fn get(&self, led: u8) -> bool {
        unsafe {
            match led {
                0 => self.gpiod.odr().read().odr12().bit(),
                1 => self.gpiod.odr().read().odr13().bit(),
                2 => self.gpiod.odr().read().odr14().bit(),
                3 => self.gpiod.odr().read().odr15().bit(),
                _ => false,
            }
        }
    }

    pub fn toggle(&mut self, led: u8) {
        unsafe {
            match led {
                0 => {
                    let current: bool = self.gpiod.odr().read().odr12().bit();
                    if current {
                        self.gpiod.bsrr().write(|w| w.br12().set_bit());
                    } else {
                        self.gpiod.bsrr().write(|w| w.bs12().set_bit());
                    }
                },
                1 => {
                    let current: bool = self.gpiod.odr().read().odr13().bit();
                    if current {
                        self.gpiod.bsrr().write(|w| w.br13().set_bit());
                    } else {
                        self.gpiod.bsrr().write(|w| w.bs13().set_bit());
                    }
                },
                2 => {
                    let current: bool = self.gpiod.odr().read().odr14().bit();
                    if current {
                        self.gpiod.bsrr().write(|w| w.br14().set_bit());
                    } else {
                        self.gpiod.bsrr().write(|w| w.bs14().set_bit());
                    }
                },
                3 => {
                    let current: bool = self.gpiod.odr().read().odr15().bit();
                    if current {
                        self.gpiod.bsrr().write(|w| w.br15().set_bit());
                    } else {
                        self.gpiod.bsrr().write(|w| w.bs15().set_bit());
                    }
                },
                _ => {}
            }
        }
    }

    pub fn toggle_all(&mut self) {
        for led in 0..4 {
            self.toggle(led);
        }
    }

    pub fn set_all(&mut self, state: bool) {
        for led in 0..4 {
            self.set(led, state);
        }
    }
}