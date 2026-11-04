from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class AcceptanceCriteria:
    adc_overruns_max: int = 0
    dma_errors_max: int = 0
    lost_samples_max: int = 0
    wav_verification_required: str = "PASS"
    storage_recovery_required: str = "PASS"
    stop2_verified_required: bool = True
    boot_verified_required: bool = True
    missing_data_status: str = "pending"

    def as_dict(self) -> Dict[str, Any]:
        return {
            "adc_overruns_max": self.adc_overruns_max,
            "dma_errors_max": self.dma_errors_max,
            "lost_samples_max": self.lost_samples_max,
            "wav_verification_required": self.wav_verification_required,
            "storage_recovery_required": self.storage_recovery_required,
            "stop2_verified_required": self.stop2_verified_required,
            "boot_verified_required": self.boot_verified_required,
            "missing_data_status": self.missing_data_status,
        }

    def evaluate(self, metrics: Dict[str, Any]) -> List[str]:
        warnings: List[str] = []

        # Boot & Reboot Verification
        if metrics.get("reboot_error"):
            warnings.append(f"MCU reboot aborted or failed: {metrics['reboot_error']}")
        if metrics.get("boot_error"):
            warnings.append(f"Boot verification failed: {metrics['boot_error']}")
        if self.boot_verified_required and metrics.get("boot_verified") is False:
            warnings.append("Boot verification failed: boot banner or system initialization milestone was not detected.")

        # Sample Accounting Verification
        expected = metrics.get("expected_samples")
        acquired = metrics.get("acquired_samples")
        processed = metrics.get("processed_samples")
        written = metrics.get("samples_written")
        lost = metrics.get("lost_samples")
        boundary = float(metrics.get("boundary_samples_discarded") or metrics.get("boundary_discarded_samples") or 0.0)

        if expected is not None and acquired is not None:
            exp_val = float(expected)
            acq_val = float(acquired)
            if acq_val < exp_val:
                deficit = int(exp_val - acq_val)
                warnings.append(
                    f"Sample shortfall: expected {int(exp_val)} samples but only acquired {int(acq_val)} "
                    f"({deficit} expected samples not acquired; session incomplete)."
                )
            elif acq_val > (exp_val + boundary):
                warnings.append(
                    f"Sample counter mismatch: acquired samples ({int(acq_val)}) exceed expected samples ({int(exp_val)}) + boundary ({int(boundary)})."
                )

        if acquired is not None and processed is not None:
            acq_val = float(acquired)
            prc_val = float(processed)
            if prc_val < acq_val:
                unprocessed = int(acq_val - prc_val)
                warnings.append(
                    f"Sample processing deficit: {unprocessed} acquired samples were not processed."
                )
            elif prc_val > acq_val:
                warnings.append(
                    f"Sample counter mismatch: processed samples ({int(prc_val)}) exceed acquired samples ({int(acq_val)})."
                )

        if processed is not None and written is not None:
            prc_val = float(processed)
            wrt_val = float(written)
            if (wrt_val + boundary) < prc_val:
                unwritten = int(prc_val - (wrt_val + boundary))
                warnings.append(
                    f"Sample storage deficit: {unwritten} processed samples were not written to SD."
                )
            elif wrt_val > prc_val:
                warnings.append(
                    f"Sample counter mismatch: written samples ({int(wrt_val)}) exceed processed samples ({int(prc_val)})."
                )

        if lost is not None and int(float(lost)) > self.lost_samples_max:
            warnings.append(f"Active capture samples lost: {int(float(lost))} samples exceed threshold {self.lost_samples_max}.")
        if metrics.get("adc_overruns") is not None and int(metrics["adc_overruns"]) > self.adc_overruns_max:
            warnings.append(f"ADC overruns {metrics['adc_overruns']} exceed threshold {self.adc_overruns_max}.")
        if metrics.get("dma_errors") is not None and int(metrics["dma_errors"]) > self.dma_errors_max:
            warnings.append(f"DMA errors {metrics['dma_errors']} exceed threshold {self.dma_errors_max}.")
        if metrics.get("wav_verification") is not None and str(metrics["wav_verification"]).upper() != self.wav_verification_required.upper():
            warnings.append(f"WAV verification {metrics['wav_verification']} does not meet required {self.wav_verification_required}.")
        if metrics.get("storage_recovery") is not None and str(metrics["storage_recovery"]).upper() != self.storage_recovery_required.upper():
            warnings.append(f"Storage recovery {metrics['storage_recovery']} does not meet required {self.storage_recovery_required}.")
        if self.stop2_verified_required:
            if metrics.get("stop2_verified") is False:
                missing = metrics.get("stop2_missing_evidence", [])
                missing_str = f" ({'; '.join(missing)})" if missing else ""
                warnings.append(f"Stop2 deep sleep unverified: missing required multi-stage lifecycle evidence{missing_str}.")
            elif metrics.get("stop2_entry") is not None and str(metrics["stop2_entry"]).upper() not in {"PASS"}:
                warnings.append("Stop2 entry result is not PASS; hardware verification required before marking sleep test passed.")
        return warnings


DEFAULT_ACCEPTANCE_CRITERIA = AcceptanceCriteria()
