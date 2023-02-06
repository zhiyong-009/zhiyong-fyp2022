#ifndef RAM_PATH_PLANNING_SEMAPHORE_HPP
#define RAM_PATH_PLANNING_SEMAPHORE_HPP

#include <mutex>

namespace ram_path_planning
{
class Semaphore
{
private:
  unsigned int counter_;
  std::mutex mutex_;
  std::condition_variable condition_;

public:
  inline Semaphore(unsigned int counter) :
          counter_(counter)
  {
  }

  inline void wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [&]()->bool
    { return counter_>0;});
    --counter_;
  }

  inline void signal()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++counter_;
    condition_.notify_one();
  }
};

}

#endif
