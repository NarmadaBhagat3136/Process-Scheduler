
#include <algorithm>
#include <ctype.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unistd.h>
#include <vector>

int __DBG = 0;

int __S_TRACE = 0;

int __E_TRACE = 0;

#define TRACE(level, format, ... )  do { \
        if (__DBG >= level) printf(format, __VA_ARGS__ ); } while(0)

#define E_TRACE(level, format, ... )  do { \
        if (__E_TRACE) printf(format, __VA_ARGS__ ); } while(0)

#define S_TRACE(level, format, ... )  do { \
        if (__S_TRACE) printf(format, __VA_ARGS__ ); } while(0)

using namespace std;

namespace util {

// Comparator for comparing smart pointer wrapper of an object.
template <class T> struct UniquePtrCompare {
    bool operator()(const unique_ptr<T>& a, const unique_ptr<T>& b) {
        return a->operator<(b);
    }
};

// Singleton class to maintain Random numbers.
class Random {
public:
    static void Init(const string& rfile);
    static int Next(int range);  // Must be called aftet Init() completes.
private:
    static unique_ptr<Random> instance_;
    int count_;
    int offset_;
    vector<int> random_numbers_;
};

unique_ptr<Random> Random::instance_ = NULL;

void Random::Init(const string& rfilename) {
    instance_.reset(new Random());
    ifstream rfile(rfilename);
    rfile >> instance_->count_;
    int r = 0;
    while(rfile >> r) {
        instance_->random_numbers_.push_back(r);
    }
    rfile.close();
    if (instance_->random_numbers_.size() != instance_->count_) {
        TRACE(4, "Expected random number size %d. Found: %d.\n",
            instance_->count_, instance_->random_numbers_.size());
    }
}

int Random::Next(int range) {
    if (instance_->offset_ == instance_->count_) {
        instance_->offset_ = 0;
    }
    return 1 + (instance_->random_numbers_[(instance_->offset_++) % instance_->count_] % range);
}

}  // namespace util

namespace process {

enum ProcessTransition {
    TRANS_TO_READY = 0,
    TRANS_TO_RUN,
    TRANS_TO_BLOCK,
    TRANS_TO_PREEMPT,  // Same as TRANS_TO_READY but more informative. No difference in processing.
    DONE
};

enum ProcessState {
    CREATED = 0,
    READY,
    RUNNING,
    BLOCK,
    PREEMPT,  
    // PREEMPT state is a dummy state created for matching the debug traces of provided output.
    // This state is exactly same as READY state for simulation and for process control.
    // No processing decision is made for this state.
    TERMINATED
};

string StateStr(ProcessState state) {
    switch(state) {
    case CREATED: return "CREATED";
    case READY: return "READY";
    case RUNNING: return "RUNNG";
    case BLOCK: return "BLOCK";
    case PREEMPT: return "PREEMPT";
    case TERMINATED: return "Done";
    default: return "";
    }
}

// PCB object that contains mutable state of the process and also stats corresponding to
// its execution.
// There should be at most one Process object for each pid at any given time. Copy and assignment
// contruction of Process is prohibited. Use ProcessPtr type to refer to this object post
// construction.
class Process {
public:
    Process(int pid, int arrival_time, int total_cpu_time,
        int cpu_burst, int io_burst, int max_priority) 
        : pid_(pid), arrival_time_(arrival_time), total_cpu_time_(total_cpu_time),
          cpu_burst_(cpu_burst), io_burst_(io_burst),
          static_priority_(util::Random::Next(max_priority)),
          state_(CREATED), state_start_time_(arrival_time), rem_(TC()), cb_(0), ib_(0),
          finish_time_(0), io_time_(0), cpu_waiting_time_(0), run_interval_(0) {
        priority_ = static_priority_ - 1;
    }
    const int AT() const { return arrival_time_; }
    const int TC() const { return total_cpu_time_; }
    const int CB() const { return cpu_burst_; }
    const int IB() const { return io_burst_; }
    const int P() const {return static_priority_; }
    const int pid() const { return pid_; }
    
    ProcessState state() const { return state_; }
    void state(ProcessState state) { state_ = state; }
    void prev_state(ProcessState value) { prev_state_ = value; }
    ProcessState prev_state() const { return prev_state_; }

    int state_start_time() const { return state_start_time_; }
    void state_start_time(int ts) { state_start_time_ = ts; }

    int cb() const { return cb_; }
    int TrySetNewCB() {
        if (cb_ == 0) {
            // Only set new cb if the old is expiring.
            cb_ = util::Random::Next(CB());
        }
        if (cb_ >  rem_) {
            // If cpu_brust is more than remaining then reduce cpu burst.
            cb_ = rem_;
        }
        run_interval_ = cb_; // Tentatively set run interval to cb. Schedulers can changes it.
        return cb_;
    }
    void AdvanceRunBy(int by) {
        cb_ -= by;
        rem_ -= by; 
    }

    int ib() const { return ib_; }
    int SetNewIb() {
        ib_ = util::Random::Next(IB());
        return ib_;
    }
    int rem() const { return rem_; }
    void rem(int value) { rem_ = value; }

    int priority() const { return priority_; }
    void priority(int value) { priority_ = value; }
    void ResetPriority() { priority_ = P() - 1; }

    void finish_time(int value) { finish_time_ = value; }

    void IncreaseIOTime(int amount) { io_time_ += amount; }

    void IncreaseCpuWaitTime(int amount) { cpu_waiting_time_ += amount; }

    int cpu_waiting_time() const { return cpu_waiting_time_; }

    string DebugStr() const;

    int turnaround_time() const { return finish_time_ - AT(); }

    int run_interval() const { return run_interval_; }
    void run_interval(int value) { run_interval_ = value; }

    // Only preemt if run_interval is less than cb. If they are equal then
    // scheduling the next IO burst takes precedance over preempting the process on quantum
    // expiration. Thus the process will be marked as blocked for IO instead of pre-empted.
    bool is_preemption() const {return run_interval_ < cb_; }

    bool is_done() const { return run_interval_ == rem_; }

private:

    // Disallow copy and assign construction for Process
    Process(const Process&);
    Process& operator=(const Process&);

    const int arrival_time_;  // Process arrival time.
    const int total_cpu_time_;  // Cpu needed by the process till completion.
    const int cpu_burst_;  // Max cpu burst of the process.
    const int io_burst_;  // Max IO Burst of the process.
    const int static_priority_;  // Initial priority of the proces.
    const int pid_;  // Unique ID.

    // Mutable state values.
    ProcessState state_;  // Current process state.
    int state_start_time_;  // Walltime when process entered this state.
    int cb_;  // Actual cpu burst if the process is in RUNNING state otherwise 0.
    int ib_;  // Actual io burst if the process is in BLOCKED state, otherwise 0.
    int rem_;  // Remaining CPU cycle of work for this process.
    int priority_;  // Dynamic Priority.
    int run_interval_;  // Length of time CPU will be available to process (<= cb due to preemption)
    ProcessState prev_state_;  // Previous state of process.

    // Stats
    // FT: Finishing time
    // TT: Turnaround time ( finishing time - AT )
    // IT: I/O Time ( time in blocked state)
    // CW: CPU Waiting time ( time in Ready state )
    int finish_time_;  // After process terminates, its termination time.
    int io_time_;  // Total time spent in IO  (BLOCKED state)
    int cpu_waiting_time_;  // Total time spent waiting for CPU in ready queue (READY state).
};

// Generate process Debug string.
string Process::DebugStr() const {
    stringstream buffer;
    buffer << setfill('0') << setw(4) << pid() << ": "
        << setfill(' ') << setw(4) << AT() << " "
        << setfill(' ') << setw(4) << TC() << " "
        << setfill(' ') << setw(4) << CB() << " "
        << setfill(' ') << setw(4) << IB() << " "
        << setfill(' ') << setw(1) << P();
    if (state_ == process::TERMINATED) {
        buffer << " | "
            << setfill(' ') << setw(5) << finish_time_ << " "
            << setfill(' ') << setw(5) << turnaround_time() << " "
            << setfill(' ') << setw(5) << io_time_ << " "
            << setfill(' ') << setw(5) << cpu_waiting_time_;
    }
    return buffer.str();
}

// Reference counted Process pointer.
typedef shared_ptr<process::Process> ProcessPtr;

// Ready / Run Queue of Process pointers.
typedef deque<ProcessPtr> ProcessQueue;

}  // namaespace process

namespace scheduler {

// Abstract class for scheduler.
class Scheduler {
public:
    // Adds a process (newly created or returning to ready state) to ready queue.
    // Returning true will pre-empting currently running process.
    virtual bool AddProcess(const process::ProcessPtr& process_ptr) = 0;

    // Schedules the next process to run.
    virtual process::ProcessPtr GetNextProcess() = 0;

    // Scheduler name.
    virtual string name() = 0;

    virtual ~Scheduler() {}

    // Number of priority level supported by this scheduler. Defaults to 4 if not implemented.
    virtual int max_priority() { return 4; }

protected:
    // Print scheduler trace for debugging.
    virtual string DebugStr() const = 0;
};

// Shared functionality of all scheduler that don't have priority and maintains
// single run/ ready queue. The process at the front of this queue is returned when scheduling
// next process
class NonPriorityScheduler : public Scheduler {
public:
    // Always returns the first process from the ready queue.
    virtual process::ProcessPtr GetNextProcess() override {
        S_TRACE(1, "SCHED (%d): %s\n", ready_queue_.size(), DebugStr().c_str());
        if (ready_queue_.empty()) {
            return nullptr;
        }
        auto process_ptr = ready_queue_.front();
        ready_queue_.pop_front();
        process_ptr->TrySetNewCB();
        return process_ptr;
    }

    virtual string DebugStr() const {
        stringstream ss;
        for (const auto& iter : ready_queue_) {
            ss << " " << iter->pid() << ":" << iter->state_start_time();
        }
        return ss.str();
    }

protected:
    // Ready queue.
    process::ProcessQueue ready_queue_;
};


// FCFS adds process at the end of event queue.
class FCFSScheduler : public NonPriorityScheduler {
public:
    virtual string name() { return "FCFS"; }

    virtual bool AddProcess(const process::ProcessPtr& process_ptr) override {
        ready_queue_.push_back(process_ptr);
        return false;
    }
};

// LCFS adds process at the begining of event queue.
class LCFSScheduler : public NonPriorityScheduler {
public:
    virtual string name() { return "LCFS"; }

    virtual bool AddProcess(const process::ProcessPtr& process_ptr) override {
        ready_queue_.push_front(process_ptr);
        return false;
    }
};

// Comparator function for comparing time remaining for SRTF scheduler. 
struct ProcessRTFCompare {
    bool operator()(const process::ProcessPtr& a, const process::ProcessPtr& b) {
        // Used in forward iterator to find the place in ready list after all other cb, smaller
        // or equal.
        return a->rem() <= b->rem();
    }
};

// SRTF adds process based on smallest remaining time comparator.
class SRTFScheduler : public NonPriorityScheduler {
public:
    virtual string name() { return "SRTF"; }

    virtual bool AddProcess(const process::ProcessPtr& process_ptr) override {
        const auto& position = 
            lower_bound(ready_queue_.begin(), ready_queue_.end(), process_ptr, ProcessRTFCompare());
        ready_queue_.insert(position, process_ptr);
        return false;
    }
};


// RR scheduler is a FCFS scheduler with preemption.
class RoundRobinScheduler : public FCFSScheduler {
public:
    explicit RoundRobinScheduler(const string& spec) : FCFSScheduler(), quantum_(0) {
        stringstream ss(spec.substr(1));
        ss >> quantum_;
    }
    virtual string name() { return "RR " + to_string(quantum_); }

    // Set process allowed run interval based on quantum available.
    virtual process::ProcessPtr GetNextProcess() override {
        auto p = FCFSScheduler::GetNextProcess();
        if (!p) {
            return nullptr;
        }
        if (p->run_interval() > quantum_) {
            p->run_interval(quantum_);
        }
        return p;
    }

private:
    int quantum_;

};

// Multi Queue priority level data structure for implementing priority based schedulers.
// Implemented as one queue for each priority. 
struct PrioProcessQueue {
public:
    explicit PrioProcessQueue(int levels) 
        : levels_(levels), set_priority_(levels), ready_queues_(levels), size_(0) {
            for (int i = 0; i < levels; ++i) {
                ready_queues_[i] = make_unique<process::ProcessQueue>();
            }
        }

    bool empty() const { return size_ == 0; }

    int size() const { return size_; }

    // Adds process to priority process queue. A process dynamic priority will be used
    // for inserting it into the right queue.
    void Add(process::ProcessPtr process_ptr) {
        int priority = process_ptr->priority();
        const auto& ready_queue = ready_queues_[levels_ - priority - 1];
        size_++;
        ready_queue->push_back(process_ptr);
        set_priority_[levels_ - priority - 1] = true;
    }

    // Removes first process from the highest priority queue containing process.
    process::ProcessPtr Pop() {
        if (empty()) {
            return nullptr;
        }
        // Find highest priority set ready queue.
        // Use set bit method to quickly find the process queue which is not empty.
        auto pos = find(set_priority_.begin(), set_priority_.end(), true);
        if (pos == set_priority_.end()) {
            // Should never happen.
            return nullptr;
        }
        int ffs = pos - set_priority_.begin();
        const auto& ready_queue = ready_queues_[ffs];
        process::ProcessPtr p = ready_queue->front();
        ready_queue->pop_front();
        if (ready_queue->empty()) {
            set_priority_[ffs] = false;
        }
        size_--;
        return p;
    }

    string DebugStr() const {
        stringstream ss;
        ss << "{ ";
        for (const auto& rq : ready_queues_) {
            string sep = "";
            ss << "[";
            for (const auto& p : *rq) {
                ss << sep << p->pid();
                sep = ",";
            }
            ss << "]";
        }
        ss << "}";
        return ss.str();
    }

private:
    // Process queues, one for each level.
    vector<unique_ptr<process::ProcessQueue>> ready_queues_;
    vector<bool> set_priority_;  // Implemented as bit-vector similar to bit array described in lec.
    int size_;  // Number of process in the queue.
    int levels_;  // Number of priority supported.
};

class PriorityScheduler : public Scheduler {
public:
    explicit PriorityScheduler(const string& spec) : max_priority_(4) {
        stringstream ss(spec.substr(1));
        string token;
        getline(ss, token, ':');
        quantum_ = atoi(token.c_str());
        if (getline(ss, token, ':')) {
            max_priority_ = atoi(token.c_str());
        }
        // Pointers to active queue and expired queue.
        active_queue_ = make_unique<PrioProcessQueue>(max_priority_);
        expired_queue_ = make_unique<PrioProcessQueue>(max_priority_);
    }

    // Adds process to the process queue. Also ensure process priority decays based on
    // if the process was pre-empted.
    // If process priority reach below zero and put the process in expired queue instead.
    virtual bool AddProcess(const process::ProcessPtr& process_ptr) {
        int current_priority = process_ptr->priority();
        if (process_ptr->prev_state() == process::BLOCK ||
            process_ptr->prev_state() == process::CREATED) {
            // A blocked process
            process_ptr->ResetPriority();
        } else if (process_ptr->prev_state() == process::RUNNING) {
            // A running process after expired quantum is entering Ready queue after pre-empt.
            process_ptr->priority(--current_priority);
        } else {
            // Will never happen.
        }
        if (process_ptr->priority() < 0) {
            process_ptr->ResetPriority();
            expired_queue_->Add(process_ptr);
        } else {
            active_queue_->Add(process_ptr);
        }
        return false;
    }

    // Returns front of the highest priority queue with process.
    virtual process::ProcessPtr GetNextProcess() {
        S_TRACE(1, "%s\n", DebugStr().c_str());

        if (active_queue_->empty()) {
            S_TRACE(1, "%s\n", "switched queues");
            Swap();
        }
        if (active_queue_->empty() && expired_queue_->empty()) {
            return nullptr;
        }

        auto p = active_queue_->Pop();
        p->TrySetNewCB();
        if (p->run_interval() > quantum_) {
            p->run_interval(quantum_);
        }
        return p;
    }
    virtual int max_priority() { return max_priority_; }

    virtual string name() {
        return "PRIO " + to_string(quantum_); 
    }

protected:

    virtual string DebugStr() const {
        stringstream ss;
        ss << active_queue_->DebugStr() << " : " << expired_queue_->DebugStr() << " : ";
        return ss.str();
    }

    void Swap() {
        unique_ptr<PrioProcessQueue> temp(move(active_queue_));
        active_queue_ = move(expired_queue_);
        expired_queue_ = move(temp);
    }

    int max_priority_;
    int quantum_;
    unique_ptr<PrioProcessQueue> active_queue_;
    unique_ptr<PrioProcessQueue> expired_queue_;
};


// Preemptive priority scheduler. Adding process will preempt currently running task.
class PreemptivePriorityScheduler : public PriorityScheduler {
public:
    explicit PreemptivePriorityScheduler(const string& spec) 
        : PriorityScheduler(spec), running_(nullptr) { }

    virtual string name() {
        return "PREPRIO " + to_string(quantum_); 
    }

    // Adds a process to ready state. If the process has higher dynamic priority than currently
    // executing process then the current process will be pre-empted.
    // Only preempt if the process is running beyond current time.
    virtual bool AddProcess(const process::ProcessPtr& process_ptr) {
        PriorityScheduler::AddProcess(process_ptr);
        // Check if currently running process need to be pre-empted.
        if (running_ && running_->state() == process::RUNNING &&
            running_->priority() < process_ptr->priority()) {
            // Verify that it is okay to not check for multiple preemption at same TS.
            // Ensure that the current process run is not already ending.
            int running_task_completion_time =
                running_->state_start_time()  + running_->run_interval();
            int current_time = process_ptr->state_start_time();
            if (running_task_completion_time > current_time) {
                running_.reset();  // No process running now.
                return true;
            }
            return false;
        }
        return false;
    }
    virtual process::ProcessPtr GetNextProcess() {
        // Maintain a currently running process that gets updated.
        // Note that process ptr can be mutated outside of scheduler. So, before using its value
        // ensure that the process is running.
        // Also it may be nullptr if no process is running.
        running_ = PriorityScheduler::GetNextProcess();
        return running_;
    }
protected:
    process::ProcessPtr running_;  // Currently running process. May get preempting.
};

// Factory class to build schedulers.
class SchedulerFactory {
public:
    static unique_ptr<Scheduler> MakeScheduler(string spec) {
        switch(spec[0]){
        case 'F': return make_unique<FCFSScheduler>();
        case 'L': return make_unique<LCFSScheduler>();
        case 'S': return make_unique<SRTFScheduler>();
        case 'R': return make_unique<RoundRobinScheduler>(spec);
        case 'P': return make_unique<PriorityScheduler>(spec);
        case 'E': return make_unique<PreemptivePriorityScheduler>(spec);
        default: return nullptr;
        }
    }
};

}  // namespace scheduler


// Start by reading in the input file and creating Process objects. 
namespace simulation {

// Container for holding simulation events. And event is 
// a pointer to process, next state to transition to and the timestamp when event
// will be processed by simulation.
class Event {
public:
    Event(const process::ProcessPtr& process,
            process::ProcessState next_state, int timestamp)
        : process_(process), timestamp_(timestamp), next_state_(next_state),
          event_id_(NEXT_ID_++) {}
    ~Event() { process_.reset(); }

    int timestamp() const { return timestamp_; }

    const process::ProcessPtr& process() const { return process_; }

    process::ProcessState next_state() const { return next_state_; }

    // Ordering in event queue will be enforced based on this comparator.
    bool operator< (const unique_ptr<Event>& other) {
        if (timestamp() != other->timestamp()) {
            return timestamp() < other->timestamp();
        }
        // There will be scenarios where events will have the same time stamp.
        if (process()->state() == process::CREATED &&
            other->process()->state() == process::CREATED) {
            // (a) Processes with the same arrival time should be entered into the run queue 
            // in the order of their occurrence in the input file.
            return process()->pid() < other->process()->pid();
        }
        // (c) Events with the same time stamp should be processed in the order they were generated,
        // Using event id created when event were created to order them.
        // Create a monotonicalling increasing event ID based on creation time and use it
        // for ordering
        return event_id_ < other->event_id_;
        
    }

    string DebugStr() const {
        stringstream ss;
        ss << timestamp() << ":" << process()->pid() << ":" << StateStr(next_state());
        return ss.str();
    }

    // Calculate transition based on next state.
    process::ProcessTransition Transition() {
        switch(next_state()) {
            case process::READY: return process::TRANS_TO_READY;
            case process::RUNNING: return process::TRANS_TO_RUN;
            case process::PREEMPT: return process::TRANS_TO_PREEMPT;
            case process::BLOCK: return process::TRANS_TO_BLOCK;
            case process::TERMINATED: return process::DONE;
            default: break;  // Never happen.
        }
        // Should never happen.
        return process::TRANS_TO_READY;
    }

private:
    process::ProcessPtr process_;
    const process::ProcessState next_state_;
    const int timestamp_;
    const int event_id_;  // Monotonically generated based on creation order.
    static int NEXT_ID_;
};

int Event::NEXT_ID_ = 1;


// Generic Event Queue. (It is generic to ensure testing with types other than process event).
template <class T> class DES {
public:
    void Put(unique_ptr<T>&& event) {
        // Insert into the event_queue list using T's comparer.
        bool is_created = (event->process()->state() == process::CREATED);
        if (!is_created) {
            E_TRACE(1, "  AddEvent(%s): %s ==> ",
                event->DebugStr().c_str(), this->DebugStr().c_str());
        }
        const auto& position = lower_bound(
            event_queue_.begin(), event_queue_.end(), event, util::UniquePtrCompare<T>());
        event_queue_.insert(position, move(event));
        if (!is_created) {
            E_TRACE(1, "%s\n",  this->DebugStr().c_str());
        }
    }

    const unique_ptr<T>& Peek() const {
        return event_queue_.front();
    }

    unique_ptr<T> Pop() {
        if (Empty()) {
            return NULL;
        }
        unique_ptr<T> event(move(event_queue_.front()));
        event_queue_.pop_front();
        return move(event);
    }

    template<class UnaryPredicate> void Remove(UnaryPredicate f) {
        // Remove the first event that returns true for the predicate.
        auto pos = find_if(event_queue_.begin(), event_queue_.end(), f);
        E_TRACE(1, "RemoveEvent(%d): %s ==> ", (*pos)->process()->pid(), this->DebugStr().c_str());
        if (pos != event_queue_.end()) {
            // Must always be the case.
            event_queue_.erase(pos);
        }
        E_TRACE(1, "%s\n",  this->DebugStr().c_str());
    }

    string DebugStr() const {
        stringstream ss;
        for (const auto& iter : event_queue_) {
            ss << iter->DebugStr() << " ";
        }
        return ss.str();
    }

    bool Empty() {
        return event_queue_.empty();
    }
private:
    list<unique_ptr<T>> event_queue_;
};

// Function object to find Event for PID in the event queue. This is used for implementing
// PREPRIO scheduler where events for a process can be removed.
class FindEventByPid {
public:
    explicit FindEventByPid(int pid, int min_event_ts) : pid_(pid), min_event_ts_(min_event_ts) {}
    
    bool operator()(const unique_ptr<Event>& v) {
        return v->process()->pid() == pid_ && v->timestamp() > min_event_ts_;
    }
private:
    int pid_;
    int min_event_ts_;
};

// Discrete event simulation controller. An instance of the simulation is created and
// provided with process file and scheduler specification.
class Simulation {
public:
    // Loads process into simulation event queue. No input validation is performed.
    // We assume the input is valid and contains at least 1 process. Each process contains
    // Arrival time, cpu needed, max cpu burst, max io burst.
    // filename: Process files.
    // scheduler_spec: String value of -s flag to the program.
    Simulation(const string& filename, const string& scheduler_spec);

    virtual ~Simulation() { stream_.close(); }

    // Starts the simulation.
    void Start();

    // Prints stat at the end of simulation.
    void PrintStats() {
        cout << scheduler_->name() << endl;
        double tt_time_total = 0;
        double cpu_wait_total = 0;
        int process_count = all_processes_.size();
        while(!all_processes_.empty()) {
            auto p = all_processes_.front();
            tt_time_total += static_cast<double>(p->turnaround_time());
            cpu_wait_total += static_cast<double>(p->cpu_waiting_time());
            cout << p->DebugStr() << endl;
            all_processes_.pop();
        }
        printf("SUM: %d %.2lf %.2lf %.2lf %.2lf %.3lf\n", 
            current_time_,
            100.0 * cpu_utilization_ / current_time_,
            100.0 * io_utilization_ / current_time_,
            tt_time_total / process_count,
            cpu_wait_total / process_count,
            100.0 * process_count / current_time_);
        // Summary Information - Finally print a summary for the simulation:
        // Finishing time of the last event (i.e. the last process finished execution)
        // CPU utilization (i.e. percentage (0.0 – 100.0) of time at least one process is running
        // IO utilization (i.e. percentage (0.0 – 100.0) of time at least one process is performing IO
        // Average turnaround time among processes
        // Average cpu waiting time among processes
        // Throughput of number processes per 100 time units
    }

private:

    ifstream stream_;
    const unique_ptr<DES<Event>> des_layer_;  // Event Queue.
    const unique_ptr<scheduler::Scheduler> scheduler_;  // Scheduler object.
    int current_time_;  // Current time of simulation. Updated when dequeing event.
    bool invoke_scheduler_;  // Will scheduler be invoked after processing the event.
    process::ProcessPtr running_;  // Pointer to running process.
    // A container to hold process pointers for printing states in the end.
    // This is not used for running simulation. This ensures all process pointers remain alive
    // until the end of simulation by having its ref count > 0.
    queue<process::ProcessPtr> all_processes_;

    // Simulation statistics to maintain for printing stats at the end.
    int cpu_utilization_;
    int io_utilization_;
    int blocked_process_count_;  // For blocked processes.
    int io_start_time_;
 };


Simulation::Simulation(const string& filename, const string& scheduler_spec) 
        : stream_(filename), des_layer_(make_unique<DES<Event>>()),
          scheduler_(scheduler::SchedulerFactory::MakeScheduler(scheduler_spec)),
          current_time_(0), invoke_scheduler_(false), running_(nullptr), cpu_utilization_(0),
          io_utilization_(0), blocked_process_count_(0), io_start_time_(-1) { 
    int arrival_time = 0, cpu_time = 0, cpu_burst = 0, io_burst = 0, pid = 0;
    int max_priority = scheduler_->max_priority();
    E_TRACE(1, "%s", "ShowEventQ:"); 
    while(stream_ >> arrival_time >> cpu_time >> cpu_burst >> io_burst) {
        auto p = process::ProcessPtr(
            new process::Process(pid++, arrival_time, cpu_time,
                                 cpu_burst, io_burst, max_priority));
        all_processes_.push(p);
        // DES layer takes ownership of the ptr.
        des_layer_->Put(make_unique<Event>(p, process::READY, p->AT()));
        E_TRACE(1, " %d:%d", p->AT(), p->pid());
    }
    E_TRACE(1, "%s", "\n");
}


void Simulation::Start() {
    // Simulation event processing.
    unique_ptr<Event> e = nullptr;
    while(e = move(des_layer_->Pop())) {  // Remove & Take ownership,
        const process::ProcessPtr& p = e->process();
        current_time_ = e->timestamp();
        int time_in_prev_state = current_time_ - p->state_start_time();
        process::ProcessState prev_state = p->state();
        if (prev_state == process::BLOCK) {
            blocked_process_count_--;
            if (blocked_process_count_ == 0 && io_start_time_ > -1) {
                io_utilization_ += current_time_ - io_start_time_;
                io_start_time_ = -1;
            }
        }
        if (prev_state == process::RUNNING) {
            p->run_interval(0);
        }
        switch(e->Transition()) {
        case process::TRANS_TO_PREEMPT:
            // PREEMPT move process to ready state. So just allow the handling similar to
            // ready transition.
        case process::TRANS_TO_READY: {
            p->prev_state(p->state());
            p->state(process::READY);
            p->state_start_time(current_time_);
            if (prev_state == process::RUNNING) {
                p->AdvanceRunBy(time_in_prev_state);
                cpu_utilization_ += time_in_prev_state;
                running_.reset();
            }

            if (p->cb() != 0) {
                TRACE(1, "%d %d %d: %s -> %s  cb=%d rem=%d prio=%d\n", 
                    e->timestamp(), p->pid(), time_in_prev_state,
                    StateStr(prev_state).c_str(), StateStr(p->state()).c_str(), p->cb(), p->rem(),
                    p->priority());                
            } else {
                TRACE(1, "%d %d %d: %s -> %s\n", e->timestamp(), p->pid(), time_in_prev_state,
                    StateStr(prev_state).c_str(), StateStr(p->state()).c_str());
            }
            // Call AddProcess after all PCB stats for process are updated.
            if (scheduler_->AddProcess(p) && running_ && prev_state != process::RUNNING) {
                S_TRACE(1, "Pre-empted process %d by %d\n", running_->pid(), p->pid());
                // Scheduling has resulted in pre-emption of existing running process. If
                // multiple process are trying to pre-empt the process, only one will preempt it.
                // Eject the process from the DES event queue.
                // When removing process, state is ignored and only event after current timestamp
                // are filtered.
                des_layer_->Remove(FindEventByPid(running_->pid(), current_time_));
                // Add a preempt event at current timestamp for this current process.
                des_layer_->Put(make_unique<Event>(running_, process::PREEMPT, current_time_));
                int actual_run_interval = current_time_ - running_->state_start_time();
                // Fix the PCB to reflect new run interval.
                running_->run_interval(actual_run_interval);
                // Mark that no process is running anymore.
                running_.reset();
            }
            if (!running_) {
                invoke_scheduler_ = true;
            }
            break;
        }
        case process::TRANS_TO_RUN: {
            p->prev_state(p->state());
            p->state(process::RUNNING);
            p->state_start_time(current_time_);
            running_ = p;
            // Calculate the next event (BLOCK or PREEMPT or DONE).
            // Termination takes precedence over scheduling the next IO burst
            // over preempting the process on quantum expiration.
            process::ProcessState next_state = process::BLOCK;
            if (p->is_preemption()) {
                // Process was pre-empted. Transition to ready state instead.
                next_state = process::PREEMPT;
            }
            if (p->is_done()) {
                next_state = process::TERMINATED;
            }
            // Assuming a process must run and not have zero cpu time.
            // A process transition to DONE from running state only.
            if (prev_state == process::READY) {
                p->IncreaseCpuWaitTime(time_in_prev_state);
            }
            TRACE(1, "%d %d %d: %s -> %s cb=%d rem=%d prio=%d\n", 
                e->timestamp(), p->pid(), time_in_prev_state,
                StateStr(prev_state).c_str(), StateStr(p->state()).c_str(), p->cb(), p->rem(),
                p->priority());
            des_layer_->Put(make_unique<Event>(p, next_state, p->run_interval() + current_time_));
            break;
        }
        case process::TRANS_TO_BLOCK: {
            p->prev_state(p->state());
            p->state(process::BLOCK);
            p->state_start_time(current_time_);
            int next_ib = p->SetNewIb();
            process::ProcessState next_state = process::READY;
            if (prev_state == process::RUNNING) {
                // Should always be the case.
                p->AdvanceRunBy(time_in_prev_state);
                cpu_utilization_ += time_in_prev_state;
                running_.reset();
                invoke_scheduler_ = true;
            }
            p->IncreaseIOTime(next_ib);
            blocked_process_count_++;
            if (io_start_time_ == -1) {
                io_start_time_ = current_time_;
            }
            TRACE(1, "%d %d %d: %s -> %s  ib=%d rem=%d\n", 
                e->timestamp(), p->pid(), time_in_prev_state,
                StateStr(prev_state).c_str(), StateStr(p->state()).c_str(), next_ib, p->rem());
            des_layer_->Put(make_unique<Event>(p, next_state, next_ib + current_time_));

            break;
        }
        case process::DONE: {
            p->prev_state(p->state());
            p->state(process::TERMINATED);
            p->finish_time(current_time_);
            if (prev_state == process::RUNNING) {
                // Must be the case.
                cpu_utilization_ += time_in_prev_state;
                p->AdvanceRunBy(time_in_prev_state);
                running_.reset();
                invoke_scheduler_ = true;
            }
            p->state_start_time(current_time_);
            TRACE(1, "%d %d %d: %s\n", e->timestamp(), p->pid(), time_in_prev_state,
                StateStr(p->state()).c_str());
            break;
        }
        default:
            break;
        }
        if (invoke_scheduler_) {
            if (!des_layer_->Empty() && des_layer_->Peek()->timestamp() == current_time_) {
                continue;
                // Must process all events at a given time stamp before invoking the
                // scheduler/dispatcher.
            }
            invoke_scheduler_ = false;
            if (!running_) {
                process::ProcessPtr next_process = scheduler_->GetNextProcess();
                if (!next_process) {
                    // If no process is waiting to run then do nothing.
                    continue;
                }
                des_layer_->Put(make_unique<Event>(next_process, process::RUNNING, current_time_));
            }
        }
    }   
}

}  // namespace simulation


int main (int argc, char **argv) {
    opterr = 0;
    string scheduler = "";
    char ch_flags = '\0';

    // Flag parsing.
    while ((ch_flags = getopt(argc, argv, "evts:")) != -1) {
        switch (ch_flags) {
            case 'v': __DBG = 1; break;  // Enables extra debug (event transitions).
            case 'e': __DBG = 1; __E_TRACE = 1; break;  // Enables event trace.
            case 't': __DBG = 1; __S_TRACE = 1; break;  // Enables scheduler trace.
            case 's': 
                scheduler = optarg; break;
            case '?':
                if (optopt == 's') {
                    fprintf (stderr, "Option -%s requires an argument.\n", optopt);
                } else if (isprint (optopt)) {
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                } else {
                    fprintf (stderr,
                                     "Unknown option character `\\x%x'.\n",
                                     optopt);
                }
                return 1;
            default:
                abort ();
        }
    }
    // Make scheduler.
    argc -= optind;
    if (argc != 2) {
        fprintf(stderr, "USAGE: sched [-v] [-t] [-e] [-s<schedspec>] inputfile randfile\n");
        return -1;
    }
    string inputfile(argv[optind]);
    string randfile(argv[optind + 1]);
    util::Random::Init(randfile);  // Loads random number file in memory.
    simulation::Simulation s(inputfile, scheduler);  // Create simulation object.
    s.Start();  // Run simulation.
    s.PrintStats();  // Print simulation result.
    return 0;
}
