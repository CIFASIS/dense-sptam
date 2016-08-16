#include "PointCloudQueue.hpp"

PointCloudEntry::PointCloudEntry(uint32_t seq)
  : seq_(seq)
  , current_pos_(nullptr)
  , update_pos_(nullptr)
  , cloud_(nullptr)
  , state_(CREATED)
{
}

PointCloudEntry::~PointCloudEntry()
{}

PointCloudQueue::PointCloudQueue()
{}

PointCloudQueue::~PointCloudQueue()
{}

PointCloudEntry::Ptr PointCloudQueue::getEntry(uint32_t seq, bool force)
{
    std::lock_guard<std::mutex> lock(vector_lock_);

    for (auto& it: entries_)
        if (it->get_seq() == seq)
            return it;

    if (force) {
        PointCloudEntry::Ptr entry(new PointCloudEntry(seq));
        entries_.push_back(entry);
        return entries_.back();
    }

    return nullptr;
}

size_t PointCloudQueue::size()
{
    std::lock_guard<std::mutex> lock(vector_lock_);
    return entries_.size();
}

size_t PointCloudQueue::sizeInitQueue()
{
    std::lock_guard<std::mutex> lock(init_queue_lock_);
    return init_queue_.size();
}

size_t PointCloudQueue::sizeRefineQueue()
{
    std::lock_guard<std::mutex> lock(refine_queue_lock_);
    return refine_queue_.size();
}

PointCloudEntry::Ptr PointCloudQueue::back()
{
    std::lock_guard<std::mutex> lock(vector_lock_);

    for (auto& it: entries_)
        if (it->get_cloud())
            return it;

    return nullptr;
}

PointCloudEntry::Ptr PointCloudQueue::popInit(bool remove)
{
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    init_queue_lock_.lock();

    while (init_queue_.empty()) {
        init_queue_lock_.unlock();
        empty_init_queue_cv.wait(lock);
        init_queue_lock_.lock();
    }

    PointCloudEntry::Ptr ret = init_queue_.front();

    if (remove)
        init_queue_.pop();

    init_queue_lock_.unlock();

    return ret;
}

PointCloudEntry::Ptr PointCloudQueue::popRefine(bool remove)
{
    std::mutex m;
    std::unique_lock<std::mutex> lock(m);

    refine_queue_lock_.lock();

    while (refine_queue_.empty()) {
        refine_queue_lock_.unlock();
        empty_refine_queue_cv.wait(lock);
        refine_queue_lock_.lock();
    }

    PointCloudEntry::Ptr ret = refine_queue_.front();

    if (remove)
        refine_queue_.pop();

    refine_queue_lock_.unlock();

    return ret;
}

void PointCloudQueue::schedule(PointCloudEntry::Ptr entry)
{
    switch(entry->get_state()) {
    case PointCloudEntry::CREATED:
        if (entry->get_cloud() && entry->get_update_pos()) {
            entry->set_state(PointCloudEntry::INIT_QUEUE);
            init_queue_.push(entry);
            empty_init_queue_cv.notify_all();
        }
        break;
    case PointCloudEntry::IDLE:
        if (entry->get_update_pos()) {
            entry->set_state(PointCloudEntry::REFINE_QUEUE);
            refine_queue_.push(entry);
            empty_refine_queue_cv.notify_all();
        }
        break;
    case PointCloudEntry::INIT_QUEUE:
    case PointCloudEntry::REFINE_QUEUE:
        break;
    }
}
