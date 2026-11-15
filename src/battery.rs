/// Voltage-based estimate following M5Unified's AXP192 getBatteryLevel().
/// This is not a calibrated fuel gauge; charging and load affect the voltage.
/// https://github.com/m5stack/M5Unified/blob/master/src/utility/power/AXP192_Class.inl
pub fn estimate_soc_percent(voltage_mv: f32, charge_current_ma: f32) -> u8 {
    let adc_voltage = voltage_mv / 1.1;
    let percent = if adc_voltage > 3150.0 {
        (adc_voltage - 3075.0) * 0.16
    } else if adc_voltage > 2690.0 {
        (adc_voltage - 2690.0) * 0.027
    } else {
        0.0
    };
    let charge_correction = if charge_current_ma > 8.0 { 16.0 } else { 0.0 };
    (percent - charge_correction).clamp(0.0, 100.0) as u8
}

#[cfg(test)]
mod tests {
    use super::estimate_soc_percent;

    #[test]
    fn empty_and_full_are_bounded_with_and_without_charging() {
        for current in [0.0, 85.0] {
            assert_eq!(estimate_soc_percent(2800.0, current), 0);
            assert_eq!(estimate_soc_percent(4200.0, current), 100);
        }
    }

    #[test]
    fn estimate_increases_with_voltage_and_compensates_for_charging() {
        for voltage in 2800..4400 {
            let voltage = voltage as f32;
            let resting = estimate_soc_percent(voltage, 0.0);
            let charging = estimate_soc_percent(voltage, 85.0);
            assert!(resting <= estimate_soc_percent(voltage + 1.0, 0.0));
            assert!(charging <= estimate_soc_percent(voltage + 1.0, 85.0));
            assert!(charging <= resting);
        }
        assert!((45..=47).contains(&estimate_soc_percent(3700.0, 0.0)));
    }
}
