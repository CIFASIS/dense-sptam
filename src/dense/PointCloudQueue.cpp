#include "PointCloudQueue.hpp"

PointCloudEntry::PointCloudEntry(
    uint32_t seq, PointCloudPtr cloud
) : seq_(seq)
, cloud_(cloud)
{}

PointCloudEntry::~PointCloudEntry()
{}

PointCloudQueue::PointCloudQueue()
{}

PointCloudQueue::~PointCloudQueue()
{}

void PointCloudQueue::push(PointCloudEntry entry)
{
    std::lock_guard<std::mutex> lock(queue_lock_);
    entries_.push_back(entry);
}

size_t PointCloudQueue::size()
{
    std::lock_guard<std::mutex> lock(queue_lock_);
    return entries_.size();
}
