/* MT6755 own clock-provider mappings, read-only ARMPLL diagnostics. */
struct mt6755_cpu_snapshot {
    u32 mux, big_con0, big_con1, little_con0, little_con1, big_div, little_div;
};

/* Decode only stock mt6755 mt_cpufreq.c's representable DDS/divider cases.
 * Unknown encodings remain unknown; this does not register a cpufreq policy. */
static int mt6755_cpu_decode(u32 con0, u32 con1, u32 mux, u32 divider, u32 *khz)
{
    u32 pcw = con1 & 0x001fffff;
    u32 postdiv = (con1 >> 24) & 7;
    u32 numerator = 1, denominator = 1, base, minimum;
    u64 value;

    *khz = 0;
    if (!(con0 & 1) || mux != 1 || postdiv > 3 ||
        (con1 & 0x78e00000) || (divider & ~0x1f))
        return -EINVAL;
    minimum = postdiv ? 0xa0000 : 0x9a000;
    base = postdiv ? 1040000 : 1001000;
    if (pcw < minimum || ((pcw - minimum) & 0x1fff))
        return -ERANGE;
    switch (divider) {
    case 0: case 8: case 16: case 24:
        break;
    case 9: case 10: case 11:
        numerator = 12 - divider; denominator = 4; break;
    case 17: case 18: case 19: case 20:
        numerator = 21 - divider; denominator = 5; break;
    case 25: case 26: case 27: case 28: case 29:
        numerator = 30 - divider; denominator = 6; break;
    default:
        return -EINVAL;
    }
    value = base + ((pcw - minimum) / 0x2000) * 13000;
    value >>= postdiv;
    value = value * numerator / denominator;
    *khz = (u32)value;
    return 0;
}

static void mt6755_cpu_snapshot(struct mt6755_cpu_snapshot *s)
{
    /* Own stock mt_cpufreq.c: TOP_CKMUXSEL, ARMCA15/7PLL_CON0/1,
     * INFRA_TOPCKGEN_CKDIV1_BIG/SML. These registers are not read-to-clear. */
    s->mux = readl(infracfg_base + 0x00);
    s->big_con0 = readl(apmixed_base + 0x200);
    s->big_con1 = readl(apmixed_base + 0x204);
    s->little_con0 = readl(apmixed_base + 0x210);
    s->little_con1 = readl(apmixed_base + 0x214);
    s->big_div = readl(infracfg_base + 0x24);
    s->little_div = readl(infracfg_base + 0x28);
}

static int mt6755_cpu_clock_get(char *buffer, const struct kernel_param *kp)
{
    struct mt6755_cpu_snapshot first, second;
    u32 big = 0, little = 0;
    int big_result, little_result;

    if (!of_machine_is_compatible("meizu,m681"))
        return scnprintf(buffer, PAGE_SIZE, "available=0 reason=board\n");
    if (!apmixed_base || !infracfg_base)
        return scnprintf(buffer, PAGE_SIZE, "available=0 reason=provider\n");
    mt6755_cpu_snapshot(&first);
    mt6755_cpu_snapshot(&second);
    if (memcmp(&first, &second, sizeof(first)))
        return scnprintf(buffer, PAGE_SIZE, "available=0 reason=unstable\n");
    big_result = mt6755_cpu_decode(first.big_con0, first.big_con1,
                                 (first.mux >> 4) & 3, first.big_div, &big);
    little_result = mt6755_cpu_decode(first.little_con0, first.little_con1,
                                    (first.mux >> 8) & 3, first.little_div, &little);
    return scnprintf(buffer, PAGE_SIZE,
        "available=1 source=pll-register-decode mux=%08x big_con0=%08x big_con1=%08x big_div=%08x big_valid=%u big_error=%d big_khz=%u little_con0=%08x little_con1=%08x little_div=%08x little_valid=%u little_error=%d little_khz=%u\n",
        first.mux, first.big_con0, first.big_con1, first.big_div,
        !big_result, big_result, big, first.little_con0, first.little_con1,
        first.little_div, !little_result, little_result, little);
}
static const struct kernel_param_ops mt6755_cpu_clock_ops = {
    .get = mt6755_cpu_clock_get,
};
module_param_cb(forge_cpu_clock, &mt6755_cpu_clock_ops, NULL, 0444);
