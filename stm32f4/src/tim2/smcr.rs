# [doc = "Register `SMCR` reader"] pub type R = crate :: R < SmcrSpec > ; # [doc = "Register `SMCR` writer"] pub type W = crate :: W < SmcrSpec > ; # [doc = "Slave mode selection\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] # [repr (u8)] pub enum Sms { # [doc = "0: Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal clock."] Disabled = 0 , # [doc = "1: Encoder mode 1 - Counter counts up/down on TI2FP1 edge depending on TI1FP2 level."] EncoderMode1 = 1 , # [doc = "2: Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level."] EncoderMode2 = 2 , # [doc = "3: Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input."] EncoderMode3 = 3 , # [doc = "4: Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers."] ResetMode = 4 , # [doc = "5: Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of the counter are controlled."] GatedMode = 5 , # [doc = "6: Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not reset). Only the start of the counter is controlled."] TriggerMode = 6 , # [doc = "7: External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter."] ExtClockMode = 7 , } impl From < Sms > for u8 { # [inline (always)] fn from (variant : Sms) -> Self { variant as _ } } impl crate :: FieldSpec for Sms { type Ux = u8 ; } impl crate :: IsEnum for Sms { } # [doc = "Field `SMS` reader - Slave mode selection"] pub type SmsR = crate :: FieldReader < Sms > ; impl SmsR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Sms { match self . bits { 0 => Sms :: Disabled , 1 => Sms :: EncoderMode1 , 2 => Sms :: EncoderMode2 , 3 => Sms :: EncoderMode3 , 4 => Sms :: ResetMode , 5 => Sms :: GatedMode , 6 => Sms :: TriggerMode , 7 => Sms :: ExtClockMode , _ => unreachable ! () , } } # [doc = "Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal clock."] # [inline (always)] pub fn is_disabled (& self) -> bool { * self == Sms :: Disabled } # [doc = "Encoder mode 1 - Counter counts up/down on TI2FP1 edge depending on TI1FP2 level."] # [inline (always)] pub fn is_encoder_mode_1 (& self) -> bool { * self == Sms :: EncoderMode1 } # [doc = "Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level."] # [inline (always)] pub fn is_encoder_mode_2 (& self) -> bool { * self == Sms :: EncoderMode2 } # [doc = "Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input."] # [inline (always)] pub fn is_encoder_mode_3 (& self) -> bool { * self == Sms :: EncoderMode3 } # [doc = "Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers."] # [inline (always)] pub fn is_reset_mode (& self) -> bool { * self == Sms :: ResetMode } # [doc = "Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of the counter are controlled."] # [inline (always)] pub fn is_gated_mode (& self) -> bool { * self == Sms :: GatedMode } # [doc = "Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not reset). Only the start of the counter is controlled."] # [inline (always)] pub fn is_trigger_mode (& self) -> bool { * self == Sms :: TriggerMode } # [doc = "External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter."] # [inline (always)] pub fn is_ext_clock_mode (& self) -> bool { * self == Sms :: ExtClockMode } } # [doc = "Field `SMS` writer - Slave mode selection"] pub type SmsW < 'a , REG > = crate :: FieldWriter < 'a , REG , 3 , Sms , crate :: Safe > ; impl < 'a , REG > SmsW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , REG :: Ux : From < u8 > { # [doc = "Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal clock."] # [inline (always)] pub fn disabled (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: Disabled) } # [doc = "Encoder mode 1 - Counter counts up/down on TI2FP1 edge depending on TI1FP2 level."] # [inline (always)] pub fn encoder_mode_1 (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: EncoderMode1) } # [doc = "Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level."] # [inline (always)] pub fn encoder_mode_2 (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: EncoderMode2) } # [doc = "Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges depending on the level of the other input."] # [inline (always)] pub fn encoder_mode_3 (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: EncoderMode3) } # [doc = "Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter and generates an update of the registers."] # [inline (always)] pub fn reset_mode (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: ResetMode) } # [doc = "Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of the counter are controlled."] # [inline (always)] pub fn gated_mode (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: GatedMode) } # [doc = "Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not reset). Only the start of the counter is controlled."] # [inline (always)] pub fn trigger_mode (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: TriggerMode) } # [doc = "External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter."] # [inline (always)] pub fn ext_clock_mode (self) -> & 'a mut crate :: W < REG > { self . variant (Sms :: ExtClockMode) } } # [doc = "Trigger selection\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] # [repr (u8)] pub enum Ts { # [doc = "0: Internal Trigger 0 (ITR0)"] Itr0 = 0 , # [doc = "1: Internal Trigger 1 (ITR1)"] Itr1 = 1 , # [doc = "2: Internal Trigger 2 (ITR2)"] Itr2 = 2 , # [doc = "4: TI1 Edge Detector (TI1F_ED)"] Ti1fEd = 4 , # [doc = "5: Filtered Timer Input 1 (TI1FP1)"] Ti1fp1 = 5 , # [doc = "6: Filtered Timer Input 2 (TI2FP2)"] Ti2fp2 = 6 , # [doc = "7: External Trigger input (ETRF)"] Etrf = 7 , } impl From < Ts > for u8 { # [inline (always)] fn from (variant : Ts) -> Self { variant as _ } } impl crate :: FieldSpec for Ts { type Ux = u8 ; } impl crate :: IsEnum for Ts { } # [doc = "Field `TS` reader - Trigger selection"] pub type TsR = crate :: FieldReader < Ts > ; impl TsR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Option < Ts > { match self . bits { 0 => Some (Ts :: Itr0) , 1 => Some (Ts :: Itr1) , 2 => Some (Ts :: Itr2) , 4 => Some (Ts :: Ti1fEd) , 5 => Some (Ts :: Ti1fp1) , 6 => Some (Ts :: Ti2fp2) , 7 => Some (Ts :: Etrf) , _ => None , } } # [doc = "Internal Trigger 0 (ITR0)"] # [inline (always)] pub fn is_itr0 (& self) -> bool { * self == Ts :: Itr0 } # [doc = "Internal Trigger 1 (ITR1)"] # [inline (always)] pub fn is_itr1 (& self) -> bool { * self == Ts :: Itr1 } # [doc = "Internal Trigger 2 (ITR2)"] # [inline (always)] pub fn is_itr2 (& self) -> bool { * self == Ts :: Itr2 } # [doc = "TI1 Edge Detector (TI1F_ED)"] # [inline (always)] pub fn is_ti1f_ed (& self) -> bool { * self == Ts :: Ti1fEd } # [doc = "Filtered Timer Input 1 (TI1FP1)"] # [inline (always)] pub fn is_ti1fp1 (& self) -> bool { * self == Ts :: Ti1fp1 } # [doc = "Filtered Timer Input 2 (TI2FP2)"] # [inline (always)] pub fn is_ti2fp2 (& self) -> bool { * self == Ts :: Ti2fp2 } # [doc = "External Trigger input (ETRF)"] # [inline (always)] pub fn is_etrf (& self) -> bool { * self == Ts :: Etrf } } # [doc = "Field `TS` writer - Trigger selection"] pub type TsW < 'a , REG > = crate :: FieldWriter < 'a , REG , 3 , Ts > ; impl < 'a , REG > TsW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , REG :: Ux : From < u8 > { # [doc = "Internal Trigger 0 (ITR0)"] # [inline (always)] pub fn itr0 (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Itr0) } # [doc = "Internal Trigger 1 (ITR1)"] # [inline (always)] pub fn itr1 (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Itr1) } # [doc = "Internal Trigger 2 (ITR2)"] # [inline (always)] pub fn itr2 (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Itr2) } # [doc = "TI1 Edge Detector (TI1F_ED)"] # [inline (always)] pub fn ti1f_ed (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Ti1fEd) } # [doc = "Filtered Timer Input 1 (TI1FP1)"] # [inline (always)] pub fn ti1fp1 (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Ti1fp1) } # [doc = "Filtered Timer Input 2 (TI2FP2)"] # [inline (always)] pub fn ti2fp2 (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Ti2fp2) } # [doc = "External Trigger input (ETRF)"] # [inline (always)] pub fn etrf (self) -> & 'a mut crate :: W < REG > { self . variant (Ts :: Etrf) } } # [doc = "Master/Slave mode\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] pub enum Msm { # [doc = "0: No action"] NoSync = 0 , # [doc = "1: The effect of an event on the trigger input (TRGI) is delayed to allow a perfect synchronization between the current timer and its slaves (through TRGO). It is useful if we want to synchronize several timers on a single external event."] Sync = 1 , } impl From < Msm > for bool { # [inline (always)] fn from (variant : Msm) -> Self { variant as u8 != 0 } } # [doc = "Field `MSM` reader - Master/Slave mode"] pub type MsmR = crate :: BitReader < Msm > ; impl MsmR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Msm { match self . bits { false => Msm :: NoSync , true => Msm :: Sync , } } # [doc = "No action"] # [inline (always)] pub fn is_no_sync (& self) -> bool { * self == Msm :: NoSync } # [doc = "The effect of an event on the trigger input (TRGI) is delayed to allow a perfect synchronization between the current timer and its slaves (through TRGO). It is useful if we want to synchronize several timers on a single external event."] # [inline (always)] pub fn is_sync (& self) -> bool { * self == Msm :: Sync } } # [doc = "Field `MSM` writer - Master/Slave mode"] pub type MsmW < 'a , REG > = crate :: BitWriter < 'a , REG , Msm > ; impl < 'a , REG > MsmW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , { # [doc = "No action"] # [inline (always)] pub fn no_sync (self) -> & 'a mut crate :: W < REG > { self . variant (Msm :: NoSync) } # [doc = "The effect of an event on the trigger input (TRGI) is delayed to allow a perfect synchronization between the current timer and its slaves (through TRGO). It is useful if we want to synchronize several timers on a single external event."] # [inline (always)] pub fn sync (self) -> & 'a mut crate :: W < REG > { self . variant (Msm :: Sync) } } # [doc = "External trigger filter\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] # [repr (u8)] pub enum Etf { # [doc = "0: No filter, sampling is done at fDTS"] NoFilter = 0 , # [doc = "1: fSAMPLING=fCK_INT, N=2"] FckIntN2 = 1 , # [doc = "2: fSAMPLING=fCK_INT, N=4"] FckIntN4 = 2 , # [doc = "3: fSAMPLING=fCK_INT, N=8"] FckIntN8 = 3 , # [doc = "4: fSAMPLING=fDTS/2, N=6"] FdtsDiv2N6 = 4 , # [doc = "5: fSAMPLING=fDTS/2, N=8"] FdtsDiv2N8 = 5 , # [doc = "6: fSAMPLING=fDTS/4, N=6"] FdtsDiv4N6 = 6 , # [doc = "7: fSAMPLING=fDTS/4, N=8"] FdtsDiv4N8 = 7 , # [doc = "8: fSAMPLING=fDTS/8, N=6"] FdtsDiv8N6 = 8 , # [doc = "9: fSAMPLING=fDTS/8, N=8"] FdtsDiv8N8 = 9 , # [doc = "10: fSAMPLING=fDTS/16, N=5"] FdtsDiv16N5 = 10 , # [doc = "11: fSAMPLING=fDTS/16, N=6"] FdtsDiv16N6 = 11 , # [doc = "12: fSAMPLING=fDTS/16, N=8"] FdtsDiv16N8 = 12 , # [doc = "13: fSAMPLING=fDTS/32, N=5"] FdtsDiv32N5 = 13 , # [doc = "14: fSAMPLING=fDTS/32, N=6"] FdtsDiv32N6 = 14 , # [doc = "15: fSAMPLING=fDTS/32, N=8"] FdtsDiv32N8 = 15 , } impl From < Etf > for u8 { # [inline (always)] fn from (variant : Etf) -> Self { variant as _ } } impl crate :: FieldSpec for Etf { type Ux = u8 ; } impl crate :: IsEnum for Etf { } # [doc = "Field `ETF` reader - External trigger filter"] pub type EtfR = crate :: FieldReader < Etf > ; impl EtfR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Etf { match self . bits { 0 => Etf :: NoFilter , 1 => Etf :: FckIntN2 , 2 => Etf :: FckIntN4 , 3 => Etf :: FckIntN8 , 4 => Etf :: FdtsDiv2N6 , 5 => Etf :: FdtsDiv2N8 , 6 => Etf :: FdtsDiv4N6 , 7 => Etf :: FdtsDiv4N8 , 8 => Etf :: FdtsDiv8N6 , 9 => Etf :: FdtsDiv8N8 , 10 => Etf :: FdtsDiv16N5 , 11 => Etf :: FdtsDiv16N6 , 12 => Etf :: FdtsDiv16N8 , 13 => Etf :: FdtsDiv32N5 , 14 => Etf :: FdtsDiv32N6 , 15 => Etf :: FdtsDiv32N8 , _ => unreachable ! () , } } # [doc = "No filter, sampling is done at fDTS"] # [inline (always)] pub fn is_no_filter (& self) -> bool { * self == Etf :: NoFilter } # [doc = "fSAMPLING=fCK_INT, N=2"] # [inline (always)] pub fn is_fck_int_n2 (& self) -> bool { * self == Etf :: FckIntN2 } # [doc = "fSAMPLING=fCK_INT, N=4"] # [inline (always)] pub fn is_fck_int_n4 (& self) -> bool { * self == Etf :: FckIntN4 } # [doc = "fSAMPLING=fCK_INT, N=8"] # [inline (always)] pub fn is_fck_int_n8 (& self) -> bool { * self == Etf :: FckIntN8 } # [doc = "fSAMPLING=fDTS/2, N=6"] # [inline (always)] pub fn is_fdts_div2_n6 (& self) -> bool { * self == Etf :: FdtsDiv2N6 } # [doc = "fSAMPLING=fDTS/2, N=8"] # [inline (always)] pub fn is_fdts_div2_n8 (& self) -> bool { * self == Etf :: FdtsDiv2N8 } # [doc = "fSAMPLING=fDTS/4, N=6"] # [inline (always)] pub fn is_fdts_div4_n6 (& self) -> bool { * self == Etf :: FdtsDiv4N6 } # [doc = "fSAMPLING=fDTS/4, N=8"] # [inline (always)] pub fn is_fdts_div4_n8 (& self) -> bool { * self == Etf :: FdtsDiv4N8 } # [doc = "fSAMPLING=fDTS/8, N=6"] # [inline (always)] pub fn is_fdts_div8_n6 (& self) -> bool { * self == Etf :: FdtsDiv8N6 } # [doc = "fSAMPLING=fDTS/8, N=8"] # [inline (always)] pub fn is_fdts_div8_n8 (& self) -> bool { * self == Etf :: FdtsDiv8N8 } # [doc = "fSAMPLING=fDTS/16, N=5"] # [inline (always)] pub fn is_fdts_div16_n5 (& self) -> bool { * self == Etf :: FdtsDiv16N5 } # [doc = "fSAMPLING=fDTS/16, N=6"] # [inline (always)] pub fn is_fdts_div16_n6 (& self) -> bool { * self == Etf :: FdtsDiv16N6 } # [doc = "fSAMPLING=fDTS/16, N=8"] # [inline (always)] pub fn is_fdts_div16_n8 (& self) -> bool { * self == Etf :: FdtsDiv16N8 } # [doc = "fSAMPLING=fDTS/32, N=5"] # [inline (always)] pub fn is_fdts_div32_n5 (& self) -> bool { * self == Etf :: FdtsDiv32N5 } # [doc = "fSAMPLING=fDTS/32, N=6"] # [inline (always)] pub fn is_fdts_div32_n6 (& self) -> bool { * self == Etf :: FdtsDiv32N6 } # [doc = "fSAMPLING=fDTS/32, N=8"] # [inline (always)] pub fn is_fdts_div32_n8 (& self) -> bool { * self == Etf :: FdtsDiv32N8 } } # [doc = "Field `ETF` writer - External trigger filter"] pub type EtfW < 'a , REG > = crate :: FieldWriter < 'a , REG , 4 , Etf , crate :: Safe > ; impl < 'a , REG > EtfW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , REG :: Ux : From < u8 > { # [doc = "No filter, sampling is done at fDTS"] # [inline (always)] pub fn no_filter (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: NoFilter) } # [doc = "fSAMPLING=fCK_INT, N=2"] # [inline (always)] pub fn fck_int_n2 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FckIntN2) } # [doc = "fSAMPLING=fCK_INT, N=4"] # [inline (always)] pub fn fck_int_n4 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FckIntN4) } # [doc = "fSAMPLING=fCK_INT, N=8"] # [inline (always)] pub fn fck_int_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FckIntN8) } # [doc = "fSAMPLING=fDTS/2, N=6"] # [inline (always)] pub fn fdts_div2_n6 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv2N6) } # [doc = "fSAMPLING=fDTS/2, N=8"] # [inline (always)] pub fn fdts_div2_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv2N8) } # [doc = "fSAMPLING=fDTS/4, N=6"] # [inline (always)] pub fn fdts_div4_n6 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv4N6) } # [doc = "fSAMPLING=fDTS/4, N=8"] # [inline (always)] pub fn fdts_div4_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv4N8) } # [doc = "fSAMPLING=fDTS/8, N=6"] # [inline (always)] pub fn fdts_div8_n6 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv8N6) } # [doc = "fSAMPLING=fDTS/8, N=8"] # [inline (always)] pub fn fdts_div8_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv8N8) } # [doc = "fSAMPLING=fDTS/16, N=5"] # [inline (always)] pub fn fdts_div16_n5 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv16N5) } # [doc = "fSAMPLING=fDTS/16, N=6"] # [inline (always)] pub fn fdts_div16_n6 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv16N6) } # [doc = "fSAMPLING=fDTS/16, N=8"] # [inline (always)] pub fn fdts_div16_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv16N8) } # [doc = "fSAMPLING=fDTS/32, N=5"] # [inline (always)] pub fn fdts_div32_n5 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv32N5) } # [doc = "fSAMPLING=fDTS/32, N=6"] # [inline (always)] pub fn fdts_div32_n6 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv32N6) } # [doc = "fSAMPLING=fDTS/32, N=8"] # [inline (always)] pub fn fdts_div32_n8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etf :: FdtsDiv32N8) } } # [doc = "External trigger prescaler\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] # [repr (u8)] pub enum Etps { # [doc = "0: Prescaler OFF"] Div1 = 0 , # [doc = "1: ETRP frequency divided by 2"] Div2 = 1 , # [doc = "2: ETRP frequency divided by 4"] Div4 = 2 , # [doc = "3: ETRP frequency divided by 8"] Div8 = 3 , } impl From < Etps > for u8 { # [inline (always)] fn from (variant : Etps) -> Self { variant as _ } } impl crate :: FieldSpec for Etps { type Ux = u8 ; } impl crate :: IsEnum for Etps { } # [doc = "Field `ETPS` reader - External trigger prescaler"] pub type EtpsR = crate :: FieldReader < Etps > ; impl EtpsR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Etps { match self . bits { 0 => Etps :: Div1 , 1 => Etps :: Div2 , 2 => Etps :: Div4 , 3 => Etps :: Div8 , _ => unreachable ! () , } } # [doc = "Prescaler OFF"] # [inline (always)] pub fn is_div1 (& self) -> bool { * self == Etps :: Div1 } # [doc = "ETRP frequency divided by 2"] # [inline (always)] pub fn is_div2 (& self) -> bool { * self == Etps :: Div2 } # [doc = "ETRP frequency divided by 4"] # [inline (always)] pub fn is_div4 (& self) -> bool { * self == Etps :: Div4 } # [doc = "ETRP frequency divided by 8"] # [inline (always)] pub fn is_div8 (& self) -> bool { * self == Etps :: Div8 } } # [doc = "Field `ETPS` writer - External trigger prescaler"] pub type EtpsW < 'a , REG > = crate :: FieldWriter < 'a , REG , 2 , Etps , crate :: Safe > ; impl < 'a , REG > EtpsW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , REG :: Ux : From < u8 > { # [doc = "Prescaler OFF"] # [inline (always)] pub fn div1 (self) -> & 'a mut crate :: W < REG > { self . variant (Etps :: Div1) } # [doc = "ETRP frequency divided by 2"] # [inline (always)] pub fn div2 (self) -> & 'a mut crate :: W < REG > { self . variant (Etps :: Div2) } # [doc = "ETRP frequency divided by 4"] # [inline (always)] pub fn div4 (self) -> & 'a mut crate :: W < REG > { self . variant (Etps :: Div4) } # [doc = "ETRP frequency divided by 8"] # [inline (always)] pub fn div8 (self) -> & 'a mut crate :: W < REG > { self . variant (Etps :: Div8) } } # [doc = "External clock enable\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] pub enum Ece { # [doc = "0: External clock mode 2 disabled"] Disabled = 0 , # [doc = "1: External clock mode 2 enabled. The counter is clocked by any active edge on the ETRF signal."] Enabled = 1 , } impl From < Ece > for bool { # [inline (always)] fn from (variant : Ece) -> Self { variant as u8 != 0 } } # [doc = "Field `ECE` reader - External clock enable"] pub type EceR = crate :: BitReader < Ece > ; impl EceR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Ece { match self . bits { false => Ece :: Disabled , true => Ece :: Enabled , } } # [doc = "External clock mode 2 disabled"] # [inline (always)] pub fn is_disabled (& self) -> bool { * self == Ece :: Disabled } # [doc = "External clock mode 2 enabled. The counter is clocked by any active edge on the ETRF signal."] # [inline (always)] pub fn is_enabled (& self) -> bool { * self == Ece :: Enabled } } # [doc = "Field `ECE` writer - External clock enable"] pub type EceW < 'a , REG > = crate :: BitWriter < 'a , REG , Ece > ; impl < 'a , REG > EceW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , { # [doc = "External clock mode 2 disabled"] # [inline (always)] pub fn disabled (self) -> & 'a mut crate :: W < REG > { self . variant (Ece :: Disabled) } # [doc = "External clock mode 2 enabled. The counter is clocked by any active edge on the ETRF signal."] # [inline (always)] pub fn enabled (self) -> & 'a mut crate :: W < REG > { self . variant (Ece :: Enabled) } } # [doc = "External trigger polarity\n\nValue on reset: 0"] # [derive (Clone , Copy , Debug , PartialEq , Eq)] pub enum Etp { # [doc = "0: ETR is noninverted, active at high level or rising edge"] NotInverted = 0 , # [doc = "1: ETR is inverted, active at low level or falling edge"] Inverted = 1 , } impl From < Etp > for bool { # [inline (always)] fn from (variant : Etp) -> Self { variant as u8 != 0 } } # [doc = "Field `ETP` reader - External trigger polarity"] pub type EtpR = crate :: BitReader < Etp > ; impl EtpR { # [doc = "Get enumerated values variant"] # [inline (always)] pub const fn variant (& self) -> Etp { match self . bits { false => Etp :: NotInverted , true => Etp :: Inverted , } } # [doc = "ETR is noninverted, active at high level or rising edge"] # [inline (always)] pub fn is_not_inverted (& self) -> bool { * self == Etp :: NotInverted } # [doc = "ETR is inverted, active at low level or falling edge"] # [inline (always)] pub fn is_inverted (& self) -> bool { * self == Etp :: Inverted } } # [doc = "Field `ETP` writer - External trigger polarity"] pub type EtpW < 'a , REG > = crate :: BitWriter < 'a , REG , Etp > ; impl < 'a , REG > EtpW < 'a , REG > where REG : crate :: Writable + crate :: RegisterSpec , { # [doc = "ETR is noninverted, active at high level or rising edge"] # [inline (always)] pub fn not_inverted (self) -> & 'a mut crate :: W < REG > { self . variant (Etp :: NotInverted) } # [doc = "ETR is inverted, active at low level or falling edge"] # [inline (always)] pub fn inverted (self) -> & 'a mut crate :: W < REG > { self . variant (Etp :: Inverted) } } impl R { # [doc = "Bits 0:2 - Slave mode selection"] # [inline (always)] pub fn sms (& self) -> SmsR { SmsR :: new ((self . bits & 7) as u8) } # [doc = "Bits 4:6 - Trigger selection"] # [inline (always)] pub fn ts (& self) -> TsR { TsR :: new (((self . bits >> 4) & 7) as u8) } # [doc = "Bit 7 - Master/Slave mode"] # [inline (always)] pub fn msm (& self) -> MsmR { MsmR :: new (((self . bits >> 7) & 1) != 0) } # [doc = "Bits 8:11 - External trigger filter"] # [inline (always)] pub fn etf (& self) -> EtfR { EtfR :: new (((self . bits >> 8) & 0x0f) as u8) } # [doc = "Bits 12:13 - External trigger prescaler"] # [inline (always)] pub fn etps (& self) -> EtpsR { EtpsR :: new (((self . bits >> 12) & 3) as u8) } # [doc = "Bit 14 - External clock enable"] # [inline (always)] pub fn ece (& self) -> EceR { EceR :: new (((self . bits >> 14) & 1) != 0) } # [doc = "Bit 15 - External trigger polarity"] # [inline (always)] pub fn etp (& self) -> EtpR { EtpR :: new (((self . bits >> 15) & 1) != 0) } } impl W { # [doc = "Bits 0:2 - Slave mode selection"] # [inline (always)] pub fn sms (& mut self) -> SmsW < SmcrSpec > { SmsW :: new (self , 0) } # [doc = "Bits 4:6 - Trigger selection"] # [inline (always)] pub fn ts (& mut self) -> TsW < SmcrSpec > { TsW :: new (self , 4) } # [doc = "Bit 7 - Master/Slave mode"] # [inline (always)] pub fn msm (& mut self) -> MsmW < SmcrSpec > { MsmW :: new (self , 7) } # [doc = "Bits 8:11 - External trigger filter"] # [inline (always)] pub fn etf (& mut self) -> EtfW < SmcrSpec > { EtfW :: new (self , 8) } # [doc = "Bits 12:13 - External trigger prescaler"] # [inline (always)] pub fn etps (& mut self) -> EtpsW < SmcrSpec > { EtpsW :: new (self , 12) } # [doc = "Bit 14 - External clock enable"] # [inline (always)] pub fn ece (& mut self) -> EceW < SmcrSpec > { EceW :: new (self , 14) } # [doc = "Bit 15 - External trigger polarity"] # [inline (always)] pub fn etp (& mut self) -> EtpW < SmcrSpec > { EtpW :: new (self , 15) } } # [doc = "slave mode control register\n\nYou can [`read`](crate::Reg::read) this register and get [`smcr::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`smcr::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."] pub struct SmcrSpec ; impl crate :: RegisterSpec for SmcrSpec { type Ux = u32 ; } # [doc = "`read()` method returns [`smcr::R`](R) reader structure"] impl crate :: Readable for SmcrSpec { } # [doc = "`write(|w| ..)` method takes [`smcr::W`](W) writer structure"] impl crate :: Writable for SmcrSpec { type Safety = crate :: Unsafe ; } # [doc = "`reset()` method sets SMCR to value 0"] impl crate :: Resettable for SmcrSpec { }