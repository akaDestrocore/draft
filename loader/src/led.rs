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
        // Enable GPIOD clock in RCC (should be done before calling this)
        
        // Configure PD12-PD15 as outputs (green, orange, red, blue LEDs)
        unsafe {
            // Configure mode register for PD12-PD15 as output (01)
            self.gpiod.moder().modify(|_, w| {
                w.moder12().output()
                 .moder13().output()
                 .moder14().output()
                 .moder15().output()
            });
            
            // Configure output type register as push-pull
            self.gpiod.otyper().modify(|_, w| {
                w.ot12().push_pull()
                 .ot13().push_pull()
                 .ot14().push_pull()
                 .ot15().push_pull()
            });
            
            // Configure output speed as low (00)
            self.gpiod.ospeedr().modify(|_, w| {
                w.ospeedr12().low_speed()
                 .ospeedr13().low_speed()
                 .ospeedr14().low_speed()
                 .ospeedr15().low_speed()
            });
            
            // Turn off all LEDs
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
    
    pub fn toggle(&mut self, led: u8) {
        unsafe {
            match led {
                0 => { let _ = self.gpiod.odr().modify(|r, w| w.odr12().bit(!r.odr12().bit())); }
                1 => { let _ = self.gpiod.odr().modify(|r, w| w.odr13().bit(!r.odr13().bit())); }
                2 => { let _ = self.gpiod.odr().modify(|r, w| w.odr14().bit(!r.odr14().bit())); }
                3 => { let _ = self.gpiod.odr().modify(|r, w| w.odr15().bit(!r.odr15().bit())); }
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