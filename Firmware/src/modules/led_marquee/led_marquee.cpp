/*
 * led_marquee.cpp
 *
 *  Created on: Mar 16, 2015
 *      Author: Lucifer
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <arch/board/board.h>

#include <visibility.h>
#include "../../lib/system/syscall.h"

extern "C" __EXPORT int led_marquee_main(int argc, char *argv[]);
int led_marquee_thread_main(int argc, char *argv[]);

static bool __task_running = false;
static bool __task_should_exit = false;

static int __task_pid = 0;

static void led_marquee_ledon(int num) {
	for (int i = 0; i < 4; i ++) {
		stm32_setled(i, (num == i) ? true : false);
	}
}

int led_marquee_thread_main(int argc, char *argv[]) {
	__task_running = true;

	stm32_ledinit();

	int led_num = 0;

	/* Main loop*/
	while (!__task_should_exit) {
		usleep(1000 * 100);	// 0.1 second
		if (led_num >= 4)
			led_num = 0;
		led_marquee_ledon(led_num);
		led_num ++;
	}
	__task_running = false;

	return 0;
}

static void led_marquee_usage(const char *reason) {
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr,
			"usage: led_marquee {start|stop|status} [-p <additional params>]\n\n");
}

int led_marquee_main(int argc, char *argv[]) {
	if (argc < 1) {
		led_marquee_usage("missing command");
		exit(1);
	}

	if (!strcmp(argv[1], "start")) {
		if (__task_running) {
			printf("led_marquee already running\n");
			/* this is not an error */
			exit(0);
		}
		__task_should_exit = false;
		__task_pid = task_spawn_cmd("led_marquee", SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5, 512, led_marquee_thread_main,
				(argv) ? (char * const *) &argv[2] : (char * const *) NULL);
		exit(0);
	}
	if (!strcmp(argv[1], "stop")) {
		__task_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {
		printf(__task_running ? "running" : "not started");
		exit(0);
	}
	led_marquee_usage("unrecognized command");
	exit(1);

	return 0;
}
