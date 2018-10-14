/* Copyright (c) Russell Gillette
 * December 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <GL/glew.h>
#include "render/RenderState.h"

/**
 * The CPU side representation of a buffer used to store indices to index into the other vertex attributes.
 * This has been made into a separate class for efficieny. It need not do as much as a vertex buffer, and
 * is stored with different commands onto the GPU.
 */
class cIBuffer
{
public:
    // takes in its index within the mesh that owns it
    // this is used to keep track of attrib array number
	cIBuffer(): mRenderID(0), mRenderState(NULL), mLocalData(NULL), mElemSize(0), mSize(0)
    {}
	cIBuffer(GLuint bufferID, cRenderState &r_state): mRenderID(bufferID), mRenderState(&r_state), mLocalData(NULL), mElemSize(0), mSize(0)
    {}
    ~cIBuffer()
    {
        if (mLocalData)
            delete [] mLocalData;
    }

    void SetRender(GLuint buffID, cRenderState &r_state)
    {
        mRenderID = buffID;
        mRenderState = &r_state;
    }
    void ResizeBuffer(int size);
    void LoadBuffer(int num_elem, int elem_size, int *data, int offset=0);
    void SyncBuffer();
    void SyncBuffer(GLuint buffer);
	int GetNumElems() const;

    int          mSize;        // the size in BYTEs of our local data store
    int          mElemSize;   // the size of an individual index
    GLubyte     *mLocalData;     // our local copy of the mesh data that we can modify and copy to the gpu

    GLuint       mRenderID;
    cRenderState *mRenderState;
};