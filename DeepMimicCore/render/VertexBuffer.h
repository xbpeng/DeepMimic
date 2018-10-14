#pragma once

#include <GL/glew.h>
#include "render/RenderState.h"
#include <string.h>

#define MAX_NUM_ATTRIB 4

/**
 * CPU side storage of the data to be stored in one VBO on the GPU. This object can handle
 * many different vertex attribute layouts.
 */
class cVertexBuffer
{
public:
    // we dont want the vbuffer owning the gl buffer id, beause then multiple
    // animations would ech require a different id (or deleting one would
    // inherently delete the gl id its working with)

	cVertexBuffer():mRenderID(0), mRenderState(NULL), mLocalData(NULL),
              mAttrInfo(NULL), mNumAttr(0), mSize(0)
    {}
	cVertexBuffer(GLuint bufferID, cRenderState &r_state):mRenderID(bufferID), mRenderState(&r_state), mLocalData(NULL),
                                       mAttrInfo(NULL), mNumAttr(0), mSize(0)
    {}
	cVertexBuffer(const cVertexBuffer &old): mRenderID(old.mRenderID), mRenderState(old.mRenderState), mNumAttr(old.mNumAttr),
                           mSize(old.mSize) {
        mLocalData = (float*) new char[mSize];
        memcpy(mLocalData, old.mLocalData, mSize);

        mAttrInfo = new tAttribInfo[mNumAttr];
        memcpy(mAttrInfo, old.mAttrInfo, sizeof(tAttribInfo)*mNumAttr);
    }

    ~cVertexBuffer()
    {
        if (mLocalData)
            delete [] mLocalData;
        if (mAttrInfo)
            delete [] mAttrInfo;
    }

    void SetRender(GLuint buffID, cRenderState &r_state)
    {
        mRenderID = buffID;
        mRenderState = &r_state;
    }

    void ResizeBuffer(int size);
    // IMPORTANT: you must load the correct VAO prior to calling this function. Since the
    // buffer object has no notion of VAO it is just setting its parameters on whatever
    // VAO is currently bound
    void LoadBuffer(int data_size, GLubyte *data, int data_offset, int num_attr, tAttribInfo *attr_info);
    void SyncBuffer();
    void SyncBuffer(GLuint buffer);
    void SyncBuffer(GLuint buffer, GLuint *size);

    int          mSize;           // the size in BYTEs of our local data store
    int          mNumAttr;       // the number of attributes per vertex
	tAttribInfo *mAttrInfo;

    float       *mLocalData;     // our local copy of the mesh data that we can modify and copy to the gpu

    GLuint       mRenderID;
    cRenderState *mRenderState;
};
