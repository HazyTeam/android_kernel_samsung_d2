/*
 *  linux/arch/m32r/kernel/process.c
 *
 *  Copyright (c) 2001, 2002  Hiroyuki Kondo, Hirokazu Takata,
 *                            Hitoshi Yamamoto
 *  Taken from sh version.
 *    Copyright (C) 1995  Linus Torvalds
 *    SuperH version:  Copyright (C) 1999, 2000  Niibe Yutaka & Kaz Kojima
 */

#undef DEBUG_PROCESS
#ifdef DEBUG_PROCESS
#define DPRINTK(fmt, args...)  printk("%s:%d:%s: " fmt, __FILE__, __LINE__, \
  __func__, ##args)
#else
#define DPRINTK(fmt, args...)
#endif

/*
 * This file handles the architecture-dependent parts of process handling..
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ptrace.h>
#include <linux/unistd.h>
#include <linux/hardirq.h>
#include <linux/rcupdate.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mmu_context.h>
#include <asm/elf.h>
#include <asm/m32r.h>

#include <linux/err.h>

/*
 * Return saved PC of a blocked thread.
 */
unsigned long thread_saved_pc(struct task_struct *tsk)
{
	return tsk->thread.lr;
}

void (*pm_power_off)(void) = NULL;
EXPORT_SYMBOL(pm_power_off);

void machine_restart(char *__unused)
{
#if defined(CONFIG_PLAT_MAPPI3)
	outw(1, (unsigned long)PLD_REBOOT);
#endif

	printk("Please push reset button!\n");
	while (1)
		cpu_relax();
}

void machine_halt(void)
{
	printk("Please push reset button!\n");
	while (1)
		cpu_relax();
}

void machine_power_off(void)
{
	/* M32R_FIXME */
}

void show_regs(struct pt_regs * regs)
{
	printk("\n");
	show_regs_print_info(KERN_DEFAULT);

	printk("BPC[%08lx]:PSW[%08lx]:LR [%08lx]:FP [%08lx]\n", \
	  regs->bpc, regs->psw, regs->lr, regs->fp);
	printk("BBPC[%08lx]:BBPSW[%08lx]:SPU[%08lx]:SPI[%08lx]\n", \
	  regs->bbpc, regs->bbpsw, regs->spu, regs->spi);
	printk("R0 [%08lx]:R1 [%08lx]:R2 [%08lx]:R3 [%08lx]\n", \
	  regs->r0, regs->r1, regs->r2, regs->r3);
	printk("R4 [%08lx]:R5 [%08lx]:R6 [%08lx]:R7 [%08lx]\n", \
	  regs->r4, regs->r5, regs->r6, regs->r7);
	printk("R8 [%08lx]:R9 [%08lx]:R10[%08lx]:R11[%08lx]\n", \
	  regs->r8, regs->r9, regs->r10, regs->r11);
	printk("R12[%08lx]\n", \
	  regs->r12);

#if defined(CONFIG_ISA_M32R2) && defined(CONFIG_ISA_DSP_LEVEL2)
	printk("ACC0H[%08lx]:ACC0L[%08lx]\n", \
	  regs->acc0h, regs->acc0l);
	printk("ACC1H[%08lx]:ACC1L[%08lx]\n", \
	  regs->acc1h, regs->acc1l);
#elif defined(CONFIG_ISA_M32R2) || defined(CONFIG_ISA_M32R)
	printk("ACCH[%08lx]:ACCL[%08lx]\n", \
	  regs->acc0h, regs->acc0l);
#else
#error unknown isa configuration
#endif
}

/*
 * Free current thread data structures etc..
 */
void exit_thread(void)
{
	/* Nothing to do. */
	DPRINTK("pid = %d\n", current->pid);
}

void flush_thread(void)
{
	DPRINTK("pid = %d\n", current->pid);
	memset(&current->thread.debug_trap, 0, sizeof(struct debug_trap));
}

void release_thread(struct task_struct *dead_task)
{
	/* do nothing */
	DPRINTK("pid = %d\n", dead_task->pid);
}

/* Fill in the fpu structure for a core dump.. */
int dump_fpu(struct pt_regs *regs, elf_fpregset_t *fpu)
{
	return 0; /* Task didn't use the fpu at all. */
}

int copy_thread(unsigned long clone_flags, unsigned long spu,
	unsigned long unused, struct task_struct *tsk, struct pt_regs *regs)
{
	struct pt_regs *childregs = task_pt_regs(tsk);
	extern void ret_from_fork(void);

	/* Copy registers */
	*childregs = *regs;

	childregs->spu = spu;
	childregs->r0 = 0;	/* Child gets zero as return value */
	regs->r0 = tsk->pid;
	tsk->thread.sp = (unsigned long)childregs;
	tsk->thread.lr = (unsigned long)ret_from_fork;

	return 0;
}

asmlinkage int sys_fork(unsigned long r0, unsigned long r1, unsigned long r2,
	unsigned long r3, unsigned long r4, unsigned long r5, unsigned long r6,
	struct pt_regs regs)
{
#ifdef CONFIG_MMU
	return do_fork(SIGCHLD, regs.spu, &regs, 0, NULL, NULL);
#else
	return -EINVAL;
#endif /* CONFIG_MMU */
}

asmlinkage int sys_clone(unsigned long clone_flags, unsigned long newsp,
			 unsigned long parent_tidptr,
			 unsigned long child_tidptr,
			 unsigned long r4, unsigned long r5, unsigned long r6,
			 struct pt_regs regs)
{
	if (!newsp)
		newsp = regs.spu;

	return do_fork(clone_flags, newsp, &regs, 0,
		       (int __user *)parent_tidptr, (int __user *)child_tidptr);
}

/*
 * This is trivial, and on the face of it looks like it
 * could equally well be done in user mode.
 *
 * Not so, for quite unobvious reasons - register pressure.
 * In user mode vfork() cannot have a stack frame, and if
 * done by calling the "clone()" system call directly, you
 * do not have enough call-clobbered registers to hold all
 * the information you need.
 */
asmlinkage int sys_vfork(unsigned long r0, unsigned long r1, unsigned long r2,
	unsigned long r3, unsigned long r4, unsigned long r5, unsigned long r6,
	struct pt_regs regs)
{
	return do_fork(CLONE_VFORK | CLONE_VM | SIGCHLD, regs.spu, &regs, 0,
			NULL, NULL);
}

/*
 * sys_execve() executes a new program.
 */
asmlinkage int sys_execve(const char __user *ufilename,
			  const char __user *const __user *uargv,
			  const char __user *const __user *uenvp,
			  unsigned long r3, unsigned long r4, unsigned long r5,
			  unsigned long r6, struct pt_regs regs)
{
	int error;
	char *filename;

	filename = getname(ufilename);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;

	error = do_execve(filename, uargv, uenvp, &regs);
	putname(filename);
out:
	return error;
}

/*
 * These bracket the sleeping functions..
 */
#define first_sched	((unsigned long) scheduling_functions_start_here)
#define last_sched	((unsigned long) scheduling_functions_end_here)

unsigned long get_wchan(struct task_struct *p)
{
	/* M32R_FIXME */
	return (0);
}
