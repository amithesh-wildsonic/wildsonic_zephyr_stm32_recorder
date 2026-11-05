import re
p = 'app/src/main.c'
t = open(p, encoding='utf-8').read()
pat = r'static void print_banner\(void\) \{[\s\S]*?printk\("Build Date: %s %s\\n", __DATE__, __TIME__\);'
rep = r'''static void print_banner(void) {
  printk("Version: %s\n", "v1.2.0");
  printk("GIT Branch: %s, SHA: %s\n", "production", "d7a4f91");
  printk("Build Date: %s %s\n", __DATE__, __TIME__);'''
assert re.search(pat, t)
open(p, 'w', encoding='utf-8').write(re.sub(pat, rep, t, count=1))
print('Success')

