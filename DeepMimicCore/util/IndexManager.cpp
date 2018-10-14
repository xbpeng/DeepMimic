#include "IndexManager.h"
#include <assert.h>

const int cIndexManager::gInvalidIndex = -1;

cIndexManager::cIndexManager()
{
	mFirstFree = 0;
}

cIndexManager::cIndexManager(int size)
{
	mFirstFree = 0;
	Resize(size);
}

cIndexManager::~cIndexManager()
{
}

int cIndexManager::GetSize() const
{
	return static_cast<int>(mIndices.size());
}

int cIndexManager::GetNumUsed() const
{
	return mFirstFree;
}

void cIndexManager::Reset()
{
	for (int i = 0; i < GetSize(); ++i)
	{
		mIndices[i] = i;
		mPos[i] = i;
	}
	mFirstFree = 0;
}

void cIndexManager::Clear()
{
	mIndices.clear();
	mPos.clear();
	mFirstFree = 0;
}

void cIndexManager::Resize(int size)
{
	int curr_size = GetSize();
	if (size < curr_size)
	{
		int curr_pos = 0;
		for (int i = 0; i < GetSize(); ++i)
		{
			int curr_idx = mIndices[i];
			if (curr_idx < size)
			{
				mIndices[curr_pos] = curr_idx;
				mPos[curr_idx] = curr_pos;

				if (i == mFirstFree)
				{
					mFirstFree = curr_pos;
				}
				++curr_pos;
			}
		}
	}

	mIndices.resize(size);
	mPos.resize(size);

	for (int i = curr_size; i < GetSize(); ++i)
	{
		mIndices[i] = i;
		mPos[i] = i;
	}
}

int cIndexManager::RequestIndex()
{
	if (IsFull())
	{
		return gInvalidIndex;
	}

	int idx = mIndices[mFirstFree];
	++mFirstFree;
	return idx;
}

void cIndexManager::FreeIndex(int idx)
{
	if(!IsFree(idx))
	{
		--mFirstFree;
		int pos = mPos[idx];
		int swap_idx = mIndices[mFirstFree];

		mIndices[mFirstFree] = idx;
		mIndices[pos] = swap_idx;

		mPos[swap_idx] = pos;
		mPos[idx] = mFirstFree;
	}
	else
	{
		assert(false); // trying to free an unused index
	}
}

bool cIndexManager::IsFree(int idx) const
{
	return mPos[idx] >= mFirstFree;
}


bool cIndexManager::IsFull() const
{
	return mFirstFree >= GetSize();
}



////////////////////////////
// Multi-Threaded
////////////////////////////

cIndexManagerMT::cIndexManagerMT()
	: cIndexManager()
{
}

cIndexManagerMT::cIndexManagerMT(int size)
	: cIndexManager(size)
{
}

cIndexManagerMT::~cIndexManagerMT()
{
}

int cIndexManagerMT::RequestIndex()
{
	int idx = gInvalidIndex;
	
	while(true)
	{
		{
			std::lock_guard<std::mutex> lock_idx(mIndexMutex);
			idx = cIndexManager::RequestIndex();
		}

		if (idx != gInvalidIndex)
		{
			break;
		}
		else
		{
			std::unique_lock<std::mutex> lock_idx(mWaitMutex);
			mCond.wait(lock_idx);
		}
	}

	return idx;
}

void cIndexManagerMT::FreeIndex(int idx)
{
	{
		std::lock_guard<std::mutex> lock_idx(mIndexMutex);
		cIndexManager::FreeIndex(idx);
	}
	mCond.notify_one();
}