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

#include <stdio.h>
#include <string.h>
#include "render/VertexBuffer.h"


void cVertexBuffer::ResizeBuffer(int size)
{
    if (size <= mSize)
        return;

    float *tmp = (float*) new GLubyte[size];

    if (mLocalData)
    {
        memcpy(tmp, mLocalData, mSize);
        delete [] mLocalData;
    }

    mSize = size;
    mLocalData = tmp;

    // nothing is actually loaded to the GPU until draw time to
    // avoid redundant updates
}

/* Each vertex has a set of attributes (such as position, normals, texture coords, etc)
 * Each attribute has a set of components (such as the x, y, and z coordinate of the position)
 *
 * They may be laid out in video memory as follows:
 *
 * Interlaced:
 *   ________________Vertex1_______________ _______Vertex2_ ...
 *  | Attribute1 | Attribute2 | Attribute3 | Attribute1 | At...
 *  | x  y  z  w | x  y  z  w | x  y  z  w | x  y  z  w | x ...
 *
 * Separate:
 *   ___Vertex1__ ___Vertex2__ ___Vertex3__ ...___Vertex1__ ___Ver...
 *  | Attribute1 | Attribute1 | Attribute1 |... Attribute2 | Attri...
 *  | x  y  z  w | x  y  z  w | x  y  z  w |... x  y  z  w | x y z...
 *
 * attr_offset is the distance to offset from the beginning of the array to get to the first
 * instance of the given attribute.
 *
 * attr_stride is the space between the start of one vertex and the start of another
 * ie from the start of one attribute and the start to the next instance of that same attribute.
 *
 * This can be even further complicated by the fact that the stride can be 0 on all arrays
 * but yet have multiple attributes by having everything within different arrays and binding
 * a new array before calling the appropriate glVertexAttribPointer call.
 * THIS IS NOT CURRENTLY HANDLED (setting up and managing the order of attrib arrays is complex)
 *
 * buffer_type specifies whether you are binding an index buffer "GL_ELEMENT_ARRAY_BUFFER" or a
 * vertex buffer "GL_ARRAY_BUFFER"
 */
void cVertexBuffer::LoadBuffer(int data_size, GLubyte *data, int data_offset, int num_attr, tAttribInfo *attr_info)
{
    // does nothing if data is already allocated
    ResizeBuffer(data_size + data_offset);

    // save parameters for future updates
    if (mNumAttr < num_attr)
    {
        delete [] mAttrInfo;
        mAttrInfo = NULL;
    }
    mNumAttr = num_attr;

    if (!mAttrInfo)
        mAttrInfo = new tAttribInfo[num_attr];

    // update our internal attribute info
    memcpy(mAttrInfo, attr_info, num_attr * sizeof(tAttribInfo));

    // update our internal data
    memcpy(mLocalData + data_offset, data, data_size);

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
void cVertexBuffer::SyncBuffer(GLuint buffer)
{
    mRenderState->SetBufferData(buffer, mSize, (unsigned char *)mLocalData);

    for (int i = 0; i < mNumAttr; ++i)
        mRenderState->SetAttributeData(mAttrInfo[i]);
}

// copy all local data to the GPU using
// stored buffer ID
void cVertexBuffer::SyncBuffer()
{ SyncBuffer(mRenderID); }