/******************************************************************************
    Copyright � 2012-2015 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#include "runtime/RuntimeImpl.h"
#include "runtime/Scheduler.h"
#include "runtime/Stack.h"
#include "runtime/Thread.h"
#include "kernel/Output.h"

unsigned int Scheduler::schedMinGranularity;
unsigned int Scheduler::defaultEpochLength;

Scheduler::Scheduler() : readyCount(0), preemption(0), resumption(0), partner(this){
  Thread* idleThread = Thread::create((vaddr)idleStack, minimumStack);
  idleThread->setAffinity(this)->setPriority(idlePriority);
  // use low-level routines, since runtime context might not exist
  idleThread->stackPointer = stackInit(idleThread->stackPointer, &Runtime::getDefaultMemoryContext(), (ptr_t)Runtime::idleLoop, this, nullptr, nullptr);
  //readyQueue[idlePriority].push_back(*idleThread);
  readyCount += 1;
  //Assignment 2 variables init
  readyTree = new Tree<ThreadNode>();
  readyTree->insert(idleThread);
  //unsigned int schedMinGranularity = 4;
  //unsigned int defaultEpochLength = 20;
  //Scheduler::setMinGran(4);
  //Scheduler::setDefaultEpoch(20);
}

//Assignment 2 start

void Scheduler::setMinGran(unsigned int toSet)   {schedMinGranularity = toSet;}
void Scheduler::setDefaultEpoch(unsigned int toSet)    {defaultEpochLength = toSet;}

unsigned int Scheduler::getMinGran()   {return schedMinGranularity;}
unsigned int Scheduler::getDefaultEpoch()    {return defaultEpochLength;}


//Assignment 2 end

static inline void unlock() {}

template<typename... Args>
static inline void unlock(BasicLock &l, Args&... a) {
  l.release();
  unlock(a...);
}

// very simple N-class prio scheduling!
template<typename... Args>
inline void Scheduler::switchThread(Scheduler* target, Args&... a) {
  KOUT::outl("calling switch thread");
  preemption += 1;
  CHECK_LOCK_MIN(sizeof...(Args));
  Thread* nextThread;
  readyLock.acquire();
  /*for (mword i = 0; i < (target ? idlePriority : maxPriority); i += 1) {
    if (!readyQueue[i].empty()) {
      nextThread = readyQueue[i].pop_front();
      readyCount -= 1;
      goto threadFound;
    }
  }*/
  if (!readyTree->empty()){
  nextThread = readyTree->popMinNode()->th;
  readyCount -= 1;
  goto threadFound;
  }
  readyLock.release();
  GENASSERT0(target);
  GENASSERT0(!sizeof...(Args));
  return;                                         // return to current thread

threadFound:
  readyLock.release();
  resumption += 1;
  Thread* currThread = Runtime::getCurrThread();
  //readyTree->insert(currThread);
  GENASSERTN(currThread && nextThread && nextThread != currThread, currThread, ' ', nextThread);

  if (target) currThread->nextScheduler = target; // yield/preempt to given processor
  else currThread->nextScheduler = this;          // suspend/resume to same processor
  unlock(a...);                                   // ...thus can unlock now
  CHECK_LOCK_COUNT(1);
  Runtime::debugS("Thread switch <", (target ? 'Y' : 'S'), ">: ", FmtHex(currThread), '(', FmtHex(currThread->stackPointer), ") to ", FmtHex(nextThread), '(', FmtHex(nextThread->stackPointer), ')');

  Runtime::MemoryContext& ctx = Runtime::getMemoryContext();

  Runtime::setCurrThread(nextThread);
//KOUT::outl("BREAKS HERE");
 Thread* prevThread = stackSwitch(currThread, target, &currThread->stackPointer, nextThread->stackPointer);

  // REMEMBER: Thread might have migrated from other processor, so 'this'
  //           might not be currThread's Scheduler object anymore.
  //           However, 'this' points to prevThread's Scheduler object.
  Runtime::postResume(false, *prevThread, ctx);

  if (currThread->state == Thread::Cancelled) {
    currThread->state = Thread::Finishing;
    switchThread(nullptr);
    unreachable();
  }
}

extern "C" Thread* postSwitch(Thread* prevThread, Scheduler* target) {
  CHECK_LOCK_COUNT(1);
  if fastpath(target) Scheduler::resume(*prevThread);
  return prevThread;
}

extern "C" void invokeThread(Thread* prevThread, Runtime::MemoryContext* ctx, funcvoid3_t func, ptr_t arg1, ptr_t arg2, ptr_t arg3) {
  //KOUT::outl("calling invokeThread method");
  Runtime::postResume(true, *prevThread, *ctx);
  func(arg1, arg2, arg3);
  Runtime::getScheduler()->terminate();
}

void Scheduler::enqueue(Thread& t) {
  KOUT::outl("calling enqueue method");
  GENASSERT1(t.priority < maxPriority, t.priority);
  readyLock.acquire();
  //readyQueue[t.priority].push_back(t);
  //A2 code
  Thread* temp = &t;
  temp->vRuntime = minvRuntime;
  readyTree->insert(temp);
  //A2 code end
  bool wake = (readyCount == 0);
  readyCount += 1;
  readyLock.release();
  Runtime::debugS("Thread ", FmtHex(&t), " queued on ", FmtHex(this));
  if (wake) Runtime::wakeUp(this);



}

void Scheduler::resume(Thread& t) {
  KOUT::outl("calling resume method");
  GENASSERT1(&t != Runtime::getCurrThread(), Runtime::getCurrThread());
  if (t.nextScheduler) t.nextScheduler->enqueue(t);
  else Runtime::getScheduler()->enqueue(t);
}

void Scheduler::preempt() {               // IRQs disabled, lock count inflated
  KOUT::outl("calling preempt method");
  Thread* currThread = Runtime::getCurrThread();
  mword curPriority = Runtime::getCurrThread()->priority;
#if TESTING_NEVER_MIGRATE
//New A2 code
/*
  currThread->vRuntime += curPriority;
  mword curRunTime = currThread->vRuntime;
    if (curRunTime >= Scheduler::getMinGran()){
      //check if leftmost vruntime is less than curRunTime
      if (readyTree->empty()){
        KOUT::outl("tree was empty");
        return;
      }
      KOUT::outl("tree was NOT empty");
      Thread* t = readyTree->readMinNode()->th;
      mword leftTime = t->vRuntime;
      if (leftTime < curRunTime){
        //Put current thread back in tree, get left node, run left node
        //readyTree->insert(currThread);
        //Thread* newThread = readyTree->popMinNode()->th;
        //minvRuntime = newThread->vRuntime;
        switchThread(target);

      }
    }*/
#else /* migration enabled */
  Scheduler* target = Runtime::getCurrThread()->getAffinity();
#if TESTING_ALWAYS_MIGRATE
  if (!target) target = partner;
#else /* simple load balancing */
  if (!target) target = (partner->readyCount + 2 < readyCount) ? partner : this;
#endif
//New A2 code
//
//DEBUG
  KOUT::outl("reached A2 code");
  currThread->vRuntime += curPriority;
  mword curRunTime = currThread->vRuntime;
  KOUT::outl("breakpoint 1");
  if (curRunTime >= Scheduler::getMinGran()){
      //check if leftmost vruntime is less than curRunTime
    /*  if (readyTree->empty()){
        //KOUT::outl("tree was empty");
        return;
      }*/
      //KOUT::outl("tree was NOT empty");
      Thread* t = readyTree->readMinNode()->th;
      KOUT::outl("breakpoint 2");
      mword leftTime = t->vRuntime;
      if (leftTime < curRunTime){
        KOUT::outl("breakpoint 3");
        minvRuntime = curRunTime;
        switchThread(target);

      }
    }
#endif
/*
//New A2 code
  //Thread* currThread = Runtime::getCurrThread();
  //Modify to use priority
  mword curPriority = Runtime::getCurrThread()->priority;
  currThread->vRuntime += curPriority;
  mword curRunTime = currThread->vRuntime;
    if (curRunTime >= Scheduler::getMinGran()){
      //check if leftmost vruntime is less than curRunTime
      if (readyTree->empty()){
        KOUT::outl("tree was empty");
        return;
      }
      KOUT::outl("tree was NOT empty");
      Thread* t = readyTree->readMinNode()->th;
      mword leftTime = t->vRuntime;
      if (leftTime < curRunTime){
        //Put current thread back in tree, get left node, run left node
        readyTree->insert(currThread);
        Thread* newThread = readyTree->popMinNode()->th;
        minvRuntime = newThread->vRuntime;
        switchThread(target);

      }
    }*/
}

void Scheduler::schedInt(){/*
  //Assignment 2 functions
  Thread* curThread = Runtime::getCurrThread();
  //Modify to use priority
  mword curPriority = Runtime::getCurrThread()->priority;
  curThread->vRuntime += curPriority;
  mword curRunTime = curThread->vRuntime;
  if (curRunTime >= Scheduler::getMinGran()){
    //check if leftmost vruntime is less than curRunTime
    if (readyTree->empty()){
      return;
    }
    Thread* t = readyTree->readMinNode()->th;
    mword leftTime = t->vRuntime;
    if (leftTime < curRunTime){
      //Put current thread back in tree, get left node, run left node
      readyTree->insert(curThread);
      Thread* newThread = readyTree->popMinNode();
      minvRuntime = newThread->vRuntime;
      //Schedule newThread to run
    }
  }
*/
}
void Scheduler::suspend(BasicLock& lk) {
  KOUT::outl("calling suspend method 1");
  Runtime::FakeLock fl;
  switchThread(nullptr, lk);
}

void Scheduler::suspend(BasicLock& lk1, BasicLock& lk2) {
  KOUT::outl("calling suspend method 2");
  Runtime::FakeLock fl;
  switchThread(nullptr, lk1, lk2);
}

void Scheduler::terminate() {
  KOUT::outl("calling terminate method");
  Runtime::RealLock rl;
  Thread* thr = Runtime::getCurrThread();
  GENASSERT1(thr->state != Thread::Blocked, thr->state);
  thr->state = Thread::Finishing;
  switchThread(nullptr);
  unreachable();
}
