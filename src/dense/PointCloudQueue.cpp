#include "PointCloudQueue.hpp"

PointCloudEntry::PointCloudEntry(uint32_t seq)
  : seq_(seq)
  , current_pos_(nullptr)
  , update_pos_(nullptr)
  , cloud_(nullptr)
{}

PointCloudEntry::~PointCloudEntry()
{}

PointCloudQueue::PointCloudQueue()
{}

PointCloudQueue::~PointCloudQueue()
{}

PointCloudEntry::Ptr PointCloudQueue::getEntry(uint32_t seq, bool force)
{
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

PointCloudEntry::Ptr PointCloudQueue::setCurrentPos(uint32_t seq, CameraPose::Ptr current_pos)
{
    std::lock_guard<std::mutex> lock(queue_lock_);

    PointCloudEntry::Ptr entry = getEntry(seq, true);
    entry->set_current_pos(current_pos);

    return entry;
}

PointCloudEntry::Ptr PointCloudQueue::setUpdatePos(uint32_t seq, CameraPose::Ptr update_pos)
{
    std::lock_guard<std::mutex> lock(queue_lock_);

    PointCloudEntry::Ptr entry = getEntry(seq, true);
    entry->set_current_pos(update_pos);

    return entry;
}

PointCloudEntry::Ptr PointCloudQueue::setCloud(uint32_t seq, PointCloudPtr cloud)
{
    std::lock_guard<std::mutex> lock(queue_lock_);

    PointCloudEntry::Ptr entry = getEntry(seq, true);
    entry->set_cloud(cloud);

    return entry;
}

size_t PointCloudQueue::size()
{
    std::lock_guard<std::mutex> lock(queue_lock_);
    return entries_.size();
}
