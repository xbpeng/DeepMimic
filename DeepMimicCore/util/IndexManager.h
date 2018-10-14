#pragma once
#include <vector>
#include <mutex>
#include <condition_variable>

class cIndexManager
{
public:
	static const int gInvalidIndex;

	cIndexManager();
	cIndexManager(int size);
	virtual ~cIndexManager();

	virtual int GetSize() const;
	virtual int GetNumUsed() const;
	virtual void Reset();
	virtual void Clear();
	virtual void Resize(int size);

	virtual int RequestIndex();
	virtual void FreeIndex(int idx);
	virtual bool IsFree(int idx) const;
	virtual bool IsFull() const;

protected:
	int mFirstFree;
	std::vector<int> mIndices;
	std::vector<int> mPos;
};

class cIndexManagerMT : public cIndexManager
{
public:
	cIndexManagerMT();
	cIndexManagerMT(int size);
	virtual ~cIndexManagerMT();

	virtual int RequestIndex();
	virtual void FreeIndex(int idx);

protected:
	std::mutex mIndexMutex;
	std::mutex mWaitMutex;
	std::condition_variable mCond;
};
