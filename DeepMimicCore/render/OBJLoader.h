/* A template to write this code was found http://weblog.benjaminsommer.com/blog/2012/02/12/a-tiny-wavefront-object-loader-part1/
 * Free to implement and use as you want
 *
 * Copyright (c) Russell Gillette
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

#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <vector>
#include <sstream>
#include <fstream>
#include <string>

using namespace std;

#define NUM_COMP_POSITION 3
#define NUM_COMP_NORMALS  3
#define NUM_COMP_TEXCOORD 2

struct FaceVertexIndex
{
    int vertex;
    int texcoord;
    int normal;
};

struct FaceVertexIndices
{
    FaceVertexIndex   *indices;
    unsigned short     nodeCount;
};

struct obj_attrib_info
{
    unsigned int attrib_number;  // which attribute within the mesh this info is for
    unsigned int attrib_size;    // total size in bytes of one attribute
    unsigned int num_comp;       // the number of components each of these atributes holds
    unsigned int data_offset;    // the number of bytes from the start of the array to the first instance of this attribute
    unsigned int data_stride;    // the number of bytes between instances of this attribute
};

class cObjLoader
{
public:
    std::vector<float>              vertices;
    std::vector<float>              normals;
    std::vector<float>              texcoords;
    std::vector<FaceVertexIndices>  faces;
    unsigned short                  faceValence;

    unsigned int m_num_attrib;
    obj_attrib_info *out_attrib_info;

    unsigned int m_data_size;
    unsigned char *out_data;

    unsigned int m_num_indices;
    int         *out_indices;

	cObjLoader(string filename)
        : out_data(NULL), out_indices(NULL), out_attrib_info(NULL),
        m_num_attrib(0), m_num_indices(0), m_data_size(0), faceValence(0)
    {
    	//<Temporary variables>
        string key;
        int a, b, c;
        string line;
        vector<FaceVertexIndex> indices = vector<FaceVertexIndex>(4);

    	ifstream ifs(filename.c_str(), ifstream::in);
    	while (ifs.good() && !ifs.eof() && std::getline(ifs, line))
        {	
            key = "";
            float x, y, z;

            stringstream str(line);
            str >> key >> ws;

    		if (key == "v")
            {
    			str >> x >> ws >> y >> ws >> z;
                vertices.push_back(x);
                vertices.push_back(y);
                vertices.push_back(z);
    		}
            else if (key == "vn")
            {
                str >> x >> ws >> y >> ws >> z;
                normals.push_back(x);
                normals.push_back(y);
                normals.push_back(z);
    		}
            else if (key == "vt")
            {
    			str >> x >> ws >> y;
                texcoords.push_back(x);
                texcoords.push_back(y);
    		}
            else if (key == "f")
            {
    			unsigned int i = 0;
                while (!str.eof())
                {
                    a = b = c = 0;
                    str >> a;

                    // guard against trailing white space
                    if (str.fail())
                        break;

                    if (str.get() == '/')
                    {
                        if (str.peek() != '/')
                        {
                            str >> b;
                            b--;
                        }
                        if (str.get() == '/')
                        {
                            str >> c;
                            c--;
                        }
                    }

                    if (i >= indices.size())
                        indices.resize(i+2);
                    indices[i].vertex   = (a-1) * NUM_COMP_POSITION;
                    indices[i].texcoord = b     * NUM_COMP_TEXCOORD;
                    indices[i].normal   = c     * NUM_COMP_NORMALS;

                    i++;
                }

                FaceVertexIndices fi;
                fi.nodeCount = i;
                fi.indices = new FaceVertexIndex[i];
                for (unsigned int j=0; j < i ; ++j)
                    fi.indices[j] = indices[j];
                faces.push_back(fi);
                faceValence = (faceValence == i || faceValence == 0) ? i : -1;
    		}
    	}
    }

    ~cObjLoader()
    {
        for (size_t i = 0; i < faces.size(); i++)
        {
            delete [] faces[i].indices;
        }
        if (out_data)
            delete [] out_data;
        if (out_indices)
            delete [] out_indices;
        if (out_attrib_info)
            delete [] out_attrib_info;
    }

    /* Export the data in a format easily read by opengl
     * All memory is assumed to have been allocated by the caller (having called query Obj) */
    void objExportGLSeparate(int &data_size, unsigned char *&data, int &num_indices, int *&indices, int &num_attrib, obj_attrib_info *&attr_info)
    {
        int v_size = vertices.size() * sizeof(vertices[0]);
        int n_size = normals.size() * sizeof(normals[0]);
        int t_size = texcoords.size() * sizeof(texcoords[0]);

        m_data_size = v_size + n_size + t_size;
        m_num_attrib = 0;
        
        if (out_data)
            delete [] out_data;
        out_data = new unsigned char[m_data_size];

        if (v_size)
        {
            // populate our array from the internal vectors
            memcpy(out_data, &vertices[0], v_size);
            m_num_attrib++;
        }
        if (n_size)
        {
            // populate our array from the internal vectors
            memcpy(&out_data[v_size], &normals[0], n_size);
            m_num_attrib++;
        }
        if (t_size)
        {
            // populate our array from the internal vectors
            memcpy(&out_data[v_size + n_size], &texcoords[0], t_size);
            m_num_attrib++;
        }

        populateAttribDataSeparate();
        indicesTriangles();

        data_size   = m_data_size;
        num_indices = m_num_indices;
        num_attrib  = m_num_attrib;

        data = out_data;
        indices = out_indices;
        attr_info = out_attrib_info;
    }

    // generate the normals for a mesh that does not have them
    // will calculate face normals
	static const int pos_dim = 3;
    void generateNormals()
    {
        // normals already exist (this function has already been
        // called or they were in the obj file)
        if (!normals.empty())
            return;

        // the array of normals should be the same size as the array of verices
        normals.resize(vertices.size());

        // iterate though faces
        for (size_t i = 0; i < faces.size(); ++i)
        {
            int num_verts = faces[i].nodeCount;
            int normalIndices[4];
            float v[4 * pos_dim];
            float n[pos_dim];

            // iterate through vertices in faces
            for (int j = 0; j < num_verts; ++j)
            {
                int v_index = faces[i].indices[j].vertex;
                // in openGL these two values must be proportional (according to the number of components)
                normalIndices[j] = v_index * NUM_COMP_NORMALS / NUM_COMP_POSITION;
                faces[i].indices[j].normal = normalIndices[j];

                // TODO: don't assume quad faces to be planar
                // memory funniness to get vec3 from float array
				v[j * pos_dim] = vertices[v_index];
				v[j * pos_dim + 1] = vertices[v_index + 1];
				v[j * pos_dim + 2] = vertices[v_index + 2];
            }

			float d0[] = { v[2 * pos_dim] - v[0], 
							v[2 * pos_dim + 1] - v[1], 
							v[2 * pos_dim + 2] - v[2] };
			float d1[] = { v[pos_dim] - v[0], 
							v[pos_dim + 1] - v[1], 
							v[pos_dim + 2] - v[2] };

            n[0] = d0[1] * d1[2] - d0[2]  * d1[1];
			n[1] = d0[2] * d1[0] - d0[0] * d1[2];
			n[2] = d0[0] * d1[1] - d0[1] * d1[0];
			double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
			n[0] /= len;
			n[1] /= len;
			n[2] /= len;

            for (int j = 0; j < num_verts; j++)
            {
				normals[normalIndices[j]] = n[0];
				normals[normalIndices[j] + 1] = n[1];
				normals[normalIndices[j] + 2] = n[2];
            }
        }
    }

private:
    void indicesTriangles()
    {
        int i_size   = faces.size();
        int num_comp = NUM_COMP_POSITION; // TODO: this needs to be dynamic (needed to normalize indices)

        // NOTE: Opengl does not allow position normal and texcoord to have different indices from vertices
        // TODO: this is terribly inefficient
        if (!i_size)
            return;

        // allocate enough memory to duplicate inner vertices
        // so that the faces can be in a gl representable structure.
        if (faces[0].nodeCount > 3)
            i_size *= (faces[0].nodeCount - 3) * 3 + 3;
        else
            i_size *= faces[0].nodeCount;

        m_num_indices = i_size;

        if (out_indices)
            delete [] out_indices;

        out_indices = new int[m_num_indices];

        for (size_t i = 0, j = 0; i < faces.size(); ++i)
        {
            for (int k = 0; k < faces[i].nodeCount; k++, j++)
            {
                if (k > 2)
                {
                    // if there are four+ vertices in a face we are going to
                    // convert to a consistant (GL_TRIANGLES) by duplicating the diagonal
                    // two indices
                    out_indices[j] = faces[i].indices[k - 3].vertex / num_comp;
                    out_indices[++j] = faces[i].indices[k - 1].vertex / num_comp;
                    ++j;
                }
                out_indices[j] = faces[i].indices[k].vertex / num_comp;
            }
        }
    }

    void populateAttribDataSeparate()
    {
        if (!m_num_attrib)
            return;
        if (out_attrib_info)
            delete [] out_attrib_info;

        out_attrib_info = new obj_attrib_info[m_num_attrib];

        int i = 0;
        if (!vertices.empty())
        {
            out_attrib_info[i].attrib_number = i;
            out_attrib_info[i].attrib_size   = NUM_COMP_POSITION * sizeof(vertices[0]);
            out_attrib_info[i].num_comp      = NUM_COMP_POSITION;
            out_attrib_info[i].data_offset   = 0;
            out_attrib_info[i].data_stride   = 0;

            ++i;
        }
        if (!normals.empty())
        {
            out_attrib_info[i].attrib_number = i;
            out_attrib_info[i].attrib_size   = NUM_COMP_NORMALS * sizeof(normals[0]);
            out_attrib_info[i].num_comp      = NUM_COMP_NORMALS;
            out_attrib_info[i].data_offset   = vertices.size() * sizeof(vertices[0]);
            out_attrib_info[i].data_stride   = 0;

            ++i;
        }
        if (!texcoords.empty())
        {
            out_attrib_info[i].attrib_number = i;
            out_attrib_info[i].attrib_size   = NUM_COMP_TEXCOORD * sizeof(texcoords[0]);
            out_attrib_info[i].num_comp      = NUM_COMP_TEXCOORD;
            out_attrib_info[i].data_offset   = normals.size() * sizeof(normals[0]) + vertices.size() * sizeof(vertices[0]);
            out_attrib_info[i].data_stride   = 0;
        }
    }
};

#endif // OBJLOADER_H