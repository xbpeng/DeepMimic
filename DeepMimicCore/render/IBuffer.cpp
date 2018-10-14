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

#include "render/IBuffer.h"

#include <string.h>
#include <cstdio>

void cIBuffer::ResizeBuffer(int size)
{
    if (size <= mSize)
        return;

    GLubyte *tmp = new GLubyte[size];
    if (mLocalData)
    {
        memcpy(tmp, mLocalData, mSize);
        delete [] mLocalData;
    }

    mLocalData = tmp;
    mSize = size;

    if (!mLocalData)
    {
        fprintf(stderr, "unable to allocate Ibuffer");
    }
}

void cIBuffer::LoadBuffer(int num_elem, int elem_size, int *data, int data_offset)
{
    if (data == NULL)
        return;

    mElemSize = elem_size;
    ResizeBuffer(num_elem * elem_size + data_offset); // does nothing if data is already allocated and large enough
    memcpy(mLocalData, data, num_elem * elem_size);     // note: don't use mSize. This method allows partial copies

#ifdef DEBUG
    GLenum err =  glGetError();
    if (err != GL_NO_ERROR)
    {
        fprintf(stderr, "Buffer Assignment Failed: %08x\n", err);
    }
#endif
}

// copy all local data to the GPU
// \param specify a buffer other than the stored buffer to allocate to
void cIBuffer::SyncBuffer(GLuint buffer)
{
    mRenderState->BindIBO(buffer);
    mRenderState->SetBufferData(buffer, mSize, (unsigned char *)mLocalData);
}

int cIBuffer::GetNumElems() const
{
	// mSize = num_elem * elem_size + data_offset
	int num_elem = mSize / mElemSize;
	return num_elem;
}

// copy all local data to the GPU using
// stored buffer ID
void cIBuffer::SyncBuffer()
{ SyncBuffer(mRenderID); }
