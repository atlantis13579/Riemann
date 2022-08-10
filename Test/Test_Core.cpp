
#include "Test.h"
#include "../Src/Core/JobSystem.h"
#include "../Src/Core/ThreadPool.h"

void thread_main(int id)
{
	printf("tid = %i\n", id);
}

void TestThreadPool()
{
	printf("Running TestThreadPool\n");
	ThreadPool pool;
	pool.Start(-1, thread_main);
	return;
}

void TestJob()
{
	printf("Running TestJob\n");
	return;
}

void TestCores()
{
	TestThreadPool();
	TestJob();
}
