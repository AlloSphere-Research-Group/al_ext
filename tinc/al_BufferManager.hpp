#ifndef AL_BUFFERMANAGER_HPP
#define AL_BUFFERMANAGER_HPP

namespace al {

template <class DataType> class BufferManager {
public:
    const int mSize;

    BufferManager(uint16_t size = 2) : mSize(size) {
        assert(size > 1);
        for (uint16_t i = 0; i < mSize; i++) {
            mData.emplace_back(std::make_shared<DataType>());
        }
    }

    std::shared_ptr<DataType> get(bool markAsUsed = true) {
        std::unique_lock<std::mutex> lk(mDataLock);
        if (markAsUsed) {
            mNewData = false;
        }
        return mData[mReadBuffer];
    }

    std::shared_ptr<DataType> getWritable() {
        std::unique_lock<std::mutex> lk(mDataLock);
        // TODO add timeout?
        while (mData[mWriteBuffer].use_count() > 1) {
            mWriteBuffer++;
            if (mWriteBuffer == mReadBuffer) {
                mWriteBuffer++;
            }
            if (mWriteBuffer >= mSize) {
                mWriteBuffer = 0;
            }
        }
        return mData[mWriteBuffer];
    }

    void doneWriting(std::shared_ptr<DataType> buffer) {
        std::unique_lock<std::mutex> lk(mDataLock);
        mReadBuffer = std::distance(mData.begin(),
                                    std::find(mData.begin(), mData.end(), buffer));
        mNewData = true;
    }

    std::shared_ptr<DataType> get(bool *isNew) {
        std::unique_lock<std::mutex> lk(mDataLock);
        if (mNewData) {
            *isNew = true;
            mNewData = false;
        }
        return mData[mReadBuffer];
    }

    bool newDataAvailable() { return mNewData; }

protected:
    std::vector<std::shared_ptr<DataType>> mData;

    std::mutex mDataLock;
    bool mNewData{false};
    uint16_t mReadBuffer{0};
    uint16_t mWriteBuffer{1};

private:
};
}

#endif // AL_BUFFERMANAGER_HPP
