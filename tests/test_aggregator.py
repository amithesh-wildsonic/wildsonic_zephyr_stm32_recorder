from __future__ import annotations

import pytest

from measurement_tool.aggregator import (
    AggregateMetric,
    MetricAggregator,
    convert_unit,
)


def test_unit_conversion_time():
    assert convert_unit(1000.0, "us", "ms") == pytest.approx(1.0)
    assert convert_unit(1000.0, "ms", "s") == pytest.approx(1.0)
    assert convert_unit(1.5, "s", "ms") == pytest.approx(1500.0)
    assert convert_unit(250.0, "µs", "us") == pytest.approx(250.0)


def test_unit_conversion_frequency():
    assert convert_unit(192000.0, "Hz", "kHz") == pytest.approx(192.0)
    assert convert_unit(48.0, "kHz", "Hz") == pytest.approx(48000.0)
    assert convert_unit(6.0, "MHz", "kHz") == pytest.approx(6000.0)


def test_unit_conversion_throughput_and_bytes():
    assert convert_unit(1024.0, "KB", "MB") == pytest.approx(1.0)
    assert convert_unit(3.0, "MB", "KB") == pytest.approx(3072.0)
    assert convert_unit(2048.0, "KB/s", "MB/s") == pytest.approx(2.0)


def test_unit_conversion_electrical():
    assert convert_unit(32000.0, "uA", "mA") == pytest.approx(32.0)
    assert convert_unit(28.5, "mA", "uA") == pytest.approx(28500.0)
    assert convert_unit(3300.0, "mV", "V") == pytest.approx(3.3)
    assert convert_unit(100.0, "mW", "W") == pytest.approx(0.1)
    assert convert_unit(1000.0, "mJ", "J") == pytest.approx(1.0)
    assert convert_unit(1.0, "J", "mJ") == pytest.approx(1000.0)


def test_unsupported_unit_raises():
    with pytest.raises(ValueError):
        convert_unit(100.0, "apples", "oranges")


def test_aggregate_metric_statistics():
    values = [10.0, 20.0, 30.0, 40.0, 50.0]
    agg = AggregateMetric.from_values(values, unit="KB/s")

    assert agg.min == 10.0
    assert agg.max == 50.0
    assert agg.avg == 30.0
    assert agg.count == 5
    assert agg.unit == "KB/s"
    assert agg.observations == values
    assert agg.format_triplet(precision=1) == "10.0 / 30.0 / 50.0"


def test_aggregate_metric_empty():
    agg = AggregateMetric.from_values([], unit="ms")
    assert agg.min is None
    assert agg.max is None
    assert agg.avg is None
    assert agg.count == 0
    assert agg.format_triplet() == "N/A"


def test_aggregate_metric_single_value():
    agg = AggregateMetric.from_values([42.5], unit="mA")
    assert agg.min == 42.5
    assert agg.max == 42.5
    assert agg.avg == 42.5
    assert agg.count == 1
    assert agg.format_triplet(precision=1) == "42.5"


def test_metric_aggregator_class():
    aggregator = MetricAggregator()
    aggregator.add_observation("sd_throughput", 2100.0, unit="KB/s")
    aggregator.add_observation("sd_throughput", 2200.0, unit="KB/s")
    aggregator.add_observation("sd_throughput", 2300.0, unit="KB/s")
    aggregator.add_scalar("wav_verified", "PASS")

    sd_agg = aggregator.get_aggregate("sd_throughput")
    assert sd_agg is not None
    assert sd_agg.avg == 2200.0
    assert sd_agg.min == 2100.0
    assert sd_agg.max == 2300.0
    assert sd_agg.count == 3

    summary = aggregator.get_summary()
    assert summary["wav_verified"] == "PASS"
    assert "sd_throughput" in summary
    assert summary["sd_throughput"]["avg"] == 2200.0

