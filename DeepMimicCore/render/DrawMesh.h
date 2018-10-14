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

#include <string>
#include <vector>
#include "render/VertexBuffer.h"
#include "render/IBuffer.h"
#include "render/RenderState.h"

/**
* Storage for all of the vertex attributes needed for rending a single mesh,
* as well as any state data specific to that mesh. (such as object model transforms)
*/
class cDrawMesh
{
public:

	cDrawMesh();
	~cDrawMesh();
	// takes the number of vbos and then a state which has enough space allocated for those vbos
	// AND an ibo
	void Init(int num_buffers);
	void Draw(GLenum primitive = GL_TRIANGLES);
	void Draw(GLenum primitive, int idx_start);
	void Draw(GLenum primitive, int idx_start, int idx_end);

	void AddBuffer(int buff_num);

	// these functions copy the the data to our internally managed memory, to modify
	// existing memory, use "GetData()
	void LoadVBuffer(unsigned int buffer_num, int size, GLubyte *data, int data_offset, int num_attr, tAttribInfo *attr_info); // load data into the specified vertex buffer
	void LoadIBuffer(int num_elem, int elem_size, int *data); // load data into the index buffer

															  // return a pointer to our internal copy of buffer i
	const float* GetData(int i) const { return mVbos[i].mLocalData; }
	const int* GetIdxData() const { return reinterpret_cast<int*>(mIbo.mLocalData); }

	int    GetNumVBO() { return static_cast<int>(mVbos.size()); }
	void   SyncGPU(unsigned int base, size_t extent = 0);     // copy the changes to our local data to the GPU
	int GetNumFaces() const;
	int GetNumVerts() const;

private:
	void ResizeBuffer(int size);            // resize the internal store for the buffer

	GLsizei  mNumElem;
	cIBuffer  mIbo;

	cRenderState    mState;
	std::vector<cVertexBuffer> mVbos;
};