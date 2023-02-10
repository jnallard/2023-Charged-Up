// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class SafetyZoner {

    private double m_min, m_max, m_excludeMin, m_excludeMax;

    public SafetyZoner(double min, double max) {
        m_min = min;
        m_max = max;
        m_excludeMin = Double.NaN;
        m_excludeMax = Double.NaN;
    }

    public void exclude(double excludeMin, double excludeMax) {
        m_excludeMin = excludeMin;
        m_excludeMax = excludeMax;
    }

    public void excludeUp(double excludeMin) {
        exclude(excludeMin, Double.NaN);
    }

    public void excludeDown(double excludeMax) {
        exclude(Double.NaN, excludeMax);
    }

    public double getSafeValue(double value) {
        var isExcludeMinSet = Double.isFinite(m_excludeMin);
        var isExcludeMaxSet = Double.isFinite(m_excludeMax);
        var clampedValue = MathUtil.clamp(value, m_min, m_max);
        // No exclusion
        if(!isExcludeMinSet && !isExcludeMaxSet) {
            return clampedValue;
        }

        // Only a min exclusion
        if(isExcludeMinSet && !isExcludeMaxSet) {
            return Math.max(clampedValue, m_excludeMin);
        }

        //only a max eclusion
        if(isExcludeMaxSet && !isExcludeMinSet) {
            return Math.min(clampedValue, m_excludeMax);
        }

        //in the exclusion zone
        if(clampedValue > m_excludeMin && clampedValue < m_excludeMax) {
            var diffUp = Math.abs(m_excludeMax - clampedValue);
            var diffDown = Math.abs(m_excludeMin - clampedValue);
            // If the distance to the excludeMin is smaller, go to excludeMin, otherwise excludeMax
            return diffDown < diffUp ? m_excludeMin : m_excludeMax;
        }

        // Not in exclusion zone
        return clampedValue;
    }

    public void resetToDefaults() {
        m_excludeMin = Double.NaN;
        m_excludeMax = Double.NaN;
    }
}
