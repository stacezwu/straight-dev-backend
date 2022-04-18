#define __SYSCALL_LL_E(x) (x)
#define __SYSCALL_LL_O(x) (x)

long __syscall0(long);
long __syscall1(long,long);
long __syscall2(long,long,long);
long __syscall3(long,long,long,long);
long __syscall4(long,long,long,long,long);
long __syscall5(long,long,long,long,long,long);
long __syscall6(long,long,long,long,long,long,long);
long __syscall7(long,long,long,long,long,long,long,long);
long __syscall8(long,long,long,long,long,long,long,long,long);

#define VDSO_USEFUL
/* We don't have a clock_gettime function.
#define VDSO_CGT_SYM "__vdso_clock_gettime"
#define VDSO_CGT_VER "LINUX_2.6" */

#define IPC_64 0
