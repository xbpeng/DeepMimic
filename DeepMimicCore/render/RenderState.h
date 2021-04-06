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
#include <vector>

struct tAttribInfo
{
    unsigned int mAttribNumber;  // which attribute within the mesh this info is for
    unsigned int mAttribSize;    // total size in bytes of one attribute
    unsigned int mNumComp;       // the number of components each of these atributes holds
    unsigned int mDataOffset;    // the number of bytes from the start of the array to the first instance of this attribute
    unsigned int mDataStride;    // the number of bytes between instances of this attribute
};

/**
 * A cRenderState object stores a collection of GPU vertex buffer objects to be used to render a
 * single mesh at a time. It also stores all of the associated state setup for
 * those buffers, such as attribute stride and offset within a vertex
 * array object. 
 */
class cRenderState
{
public:
    cRenderState()
    {
        glGenVertexArrays(1, &mVaoID);
    }
    ~cRenderState()
    {
        glDeleteVertexArrays(1, &mVaoID);

        if (!mVboIDs.empty())
            glDeleteBuffers(static_cast<GLsizei>(mVboIDs.size()), &mVboIDs[0]);
    }
    void BindVAO()
    {
        glBindVertexArray(mVaoID);
    }
    void BindVBO(unsigned int i, GLenum target = GL_ARRAY_BUFFER)
    {
        glBindBuffer(target, operator[](i));
    }

    void BindIBO(unsigned int i)
    { BindVBO(i, GL_ELEMENT_ARRAY_BUFFER); }


    void SetNextBufferData(std::size_t num_bytes, unsigned char *data)
    {
        SetBufferData(mDirty++, num_bytes, data);
    }

    void SetBufferData(unsigned int i, std::size_t num_bytes, unsigned char *data = NULL)
    {
        BindVBO(i);
        if (mBytes[i] < num_bytes)
        {
            mBytes[i] = static_cast<unsigned int>(num_bytes);
            glBufferData(GL_ARRAY_BUFFER, num_bytes, data, GL_STATIC_DRAW); // Static, because all changes happen in the edit mesh
        }
        else
            glBufferSubData(GL_ARRAY_BUFFER, 0, num_bytes, data);
    }
    void SetAttributeData(tAttribInfo &info)
    { SetAttributeData(info.mAttribNumber, info.mNumComp, info.mDataOffset, info.mDataStride); }

    void SetAttributeData(unsigned int mAttribNumber, unsigned int mNumComp, unsigned int mDataOffset, unsigned int mDataStride)
    {
        glEnableVertexAttribArray(mAttribNumber);
        glVertexAttribPointer(mAttribNumber, mNumComp, GL_FLOAT, GL_FALSE,
                              mDataStride, reinterpret_cast<GLvoid*>(mDataOffset));
    }

    GLuint operator[](unsigned int i)
    {
        if (i >= mVboIDs.size())
        {
            mVboIDs.resize(i+1, 0);
            mBytes.resize(i+1, 0);
        }
        if (mVboIDs[i] == 0)
            glGenBuffers(1, &mVboIDs[i]);
        return mVboIDs[i];
    }

    void MakeClean()
    { mDirty = 0; }

private:
    int mDirty; // used for loading to copy into appropriate buffers

    GLuint mVaoID;
    std::vector<GLuint> mVboIDs;
	std::vector<GLuint> mBytes; // used to avoid needless resizing of buffers
};