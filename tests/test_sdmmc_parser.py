from measurement_tool.parsers.sdmmc import parse_sdmmc_metrics


def test_sdmmc_parser_extracts_throughput_and_recovery_details():
    sample = """
    Active write throughput: 11.2 MB/s
    End-to-end throughput: 10.4 MB/s
    Aggregate writer throughput: 12.3 MB/s
    Average batch latency: 4.2 ms
    Worst batch latency: 7.8 ms
    SDMMC errors: 0
    Filesystem errors: 0
    Storage recovery: PASS
    """

    metrics = parse_sdmmc_metrics(sample)

    assert metrics["active_write_throughput"] == 11.2
    assert metrics["sdmmc_errors"] == 0.0
    assert metrics["filesystem_errors"] == 0.0
    assert metrics["storage_recovery"] == "PASS"
