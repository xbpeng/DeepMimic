#include "FileUtil.h"
#include <assert.h>
#include <cstdarg>
#include <memory>
#include <iostream>
#include <string.h>

FILE* cFileUtil::OpenFile(const std::string& file_name, const char* mode)
{
	return OpenFile(file_name.c_str(), mode);
}

FILE* cFileUtil::OpenFile(const char* path, const char* mode)
{
	FILE* f = nullptr;
	f = fopen(path, mode);
	if (f == nullptr)
	{
		printf("Failed to open %s!\n", path);
		assert(false); // failed to open file
	}
	return f;
}

void cFileUtil::CloseFile(FILE*& f)
{
	if (f != nullptr)
	{
		fclose(f);
		f = nullptr;
	}
}

void cFileUtil::ClearFile(const std::string& file_name)
{
	FILE* f = OpenFile(file_name, "w");
	CloseFile(f);
}

void cFileUtil::CreateFile(const std::string& file_name)
{
	ClearFile(file_name);
}

void cFileUtil::DeleteFile(const char* file_name)
{
	bool succc = remove(file_name) == 0;
	if (!succc)
	{
		printf("Failed to delete %s!\n", file_name);
		assert(false); // failed to open file
	}
}


std::string cFileUtil::RemoveExtension(const std::string& filename)
{
	size_t first_not_dot = filename.find_first_not_of('.');
	size_t last_dot = filename.find_last_of(".");
	if (last_dot == std::string::npos
		|| last_dot <= first_not_dot)
	{
		return filename;
	}
	return filename.substr(0, last_dot);
}

void cFileUtil::DeleteFile(const std::string& filename)
{
	int err = remove(filename.c_str());
	if (err != 0)
	{
		printf("Failed to delete %s!\n", filename.c_str());
		assert(false);
	}
}

long int cFileUtil::GetFileSize(const std::string& filename)
{
	// returns size in bytes
	FILE* f = OpenFile(filename.c_str(), "rb");

	if (f != NULL)
	{
		fseek(f, 0, SEEK_END);
		long int f_size = ftell(f);
		CloseFile(f);
		return f_size;
	}
	return 0;
}

std::string cFileUtil::GetExtension(const std::string& filename)
{
	// remove leading '.'
	size_t dot_idx = 0;
	for (dot_idx; dot_idx < filename.size(); ++dot_idx)
	{
		if (filename[dot_idx] != '.')
		{
			break;
		}
	}

	std::string str = filename.substr(dot_idx);
	size_t pos = str.find_last_of(".");
	if (pos == std::string::npos)
	{
		return "";
	}
	return str.substr(pos + 1);
}

std::string cFileUtil::GetFilename(const std::string& path)
{
	int idx = 0;
	for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
	{
		char curr_char = path[i];
		if (curr_char == '\\' || curr_char == '/')
		{
			idx = i + 1;
			break;
		}
	}

	std::string filename = path.substr(idx, path.size() - idx);
	filename = cFileUtil::RemoveExtension(filename);
	return filename;
}

void cFileUtil::FilterFilesByExtension(std::vector<std::string>& files, const std::string& ext)
{
	size_t i = 0;
	for (size_t j = 0; j < files.size(); ++j)
	{
		const std::string& curr_f = files[j];
		std::string curr_ext = GetExtension(curr_f);
		if (curr_ext == ext)
		{
			files[i] = curr_f;
			++i;
		}
	}
	files.resize(i);
}

bool cFileUtil::ExistsFile(const std::string& file_name)
{
	FILE* f = nullptr;
	f = fopen(file_name.c_str(), "r");
	if (f != nullptr)
	{
		fclose(f);
		return true;
	}
	return false;
}


void cFileUtil::FindLine(std::ifstream& f_stream, int line)
{
	f_stream.seekg(std::ios::beg);
	std::string str;
	int l = 0;
	while(std::getline(f_stream, str))
	{
		if (l == line - 1)
		{
			return;
		}
		++l;
	}

	throw "Failed to find line in file stream\n";
}

std::string cFileUtil::ReadTextFile(const std::string& path)
{
	FILE *file = OpenFile(path.c_str(), "rb");
	std::string text = ReadTextFile(file);
	fclose(file);
	return text;
}
/*
bool cFileUtil::ReadArray(FILE* f, const std::string& tag_beg, const std::string& tag_end, std::vector<double>& out_buffer)
{

	std::fstream f_stream("wft");
	out_buffer.clear();

	const char delims[] = " ,\t";

	std::string str;
	std::vector<char> char_array;

	bool succ = false;
	bool found = false;
	while (std::getline(f_stream, str))
	{
		if (str == tag_beg)
		{
			found = true;
		}
		else if (str == tag_end)
		{
			succ = found;
			break;
		}
		else if (found)
		{
			if (str.size() > 0)
			{
				char_array = std::vector<char>(str.begin(), str.end());
				char_array.push_back(0);

				char* p_char = NULL;
				p_char = strtok(&char_array[0], delims);

				if (p_char != nullptr)
				{
					std::string curr_tok(p_char);
					if (curr_tok.size() >= 2)
					{
						if (curr_tok[0] == '/' && curr_tok[1] == '/')
						{
							continue;
						}
					}

					double val = std::atof(p_char);
					out_buffer.push_back(val);
				}
			}
		}
	}
	//f_stream.close();

	if (!succ)
	{
		out_buffer.clear();
	}
	return succ;
}
*/

bool cFileUtil::ReadTable(const std::string& filename, std::vector<std::vector<double>>& out_buffer)
{
	std::fstream f_stream(filename);
	out_buffer.clear();

	const char delims[] = " ,\t";

	std::string str;
	std::vector<char> char_array;

	bool succ = true;
	while (std::getline(f_stream, str))
	{
		if (str.size() > 0)
		{
			std::vector<double> curr_array;

			char_array = std::vector<char>(str.begin(), str.end());
			char_array.push_back(0);

			char* p_char = NULL;
			p_char = strtok(&char_array[0], delims);

			while (p_char != nullptr)
			{
				std::string curr_tok(p_char);
				if (curr_tok.size() >= 2)
				{
					if (curr_tok[0] == '/' && curr_tok[1] == '/')
					{
						break;
					}
				}

				double val = std::atof(p_char);
				curr_array.push_back(val);
				p_char = strtok(NULL, delims);
			}

			if (curr_array.size() > 0)
			{
				out_buffer.push_back(curr_array);
			}
		}
	}

	if (!succ)
	{
		out_buffer.clear();
	}
	return succ;
}

bool cFileUtil::ReadMatrix(const std::string& filename, Eigen::MatrixXd& out_mat)
{
	std::vector<std::vector<double>> data;
	cFileUtil::ReadTable(filename, data);

	bool succ = false;
	if (data.size() > 0 && data[0].size() > 0)
	{
		int n = static_cast<int>(data.size());
		int m = static_cast<int>(data[0].size());
		out_mat.resize(n, m);

		for (int i = 0; i < n; ++i)
		{
			auto curr_row = data[i];
			int curr_m = static_cast<int>(curr_row.size());
			assert(curr_m == m);
			for (int j = 0; j < m; ++j)
			{
				out_mat(i, j) = curr_row[j];
			}
		}
		succ = true;
	}
	return succ;
}

bool cFileUtil::WriteMatrix(const Eigen::MatrixXd& mat, const std::string& out_filename)
{
	FILE* f = cFileUtil::OpenFile(out_filename, "w");
	bool succ = f != nullptr;

	if (succ)
	{
		for (int i = 0; i < mat.rows(); ++i)
		{
			for (int j = 0; j < mat.cols(); ++j)
			{
				if (j != 0)
				{
					fprintf(f, ",");
				}
				fprintf(f, "%20.10f", mat(i, j));
			}
			fprintf(f, "\n");
		}
		cFileUtil::CloseFile(f);
	}
	return succ;
}

bool cFileUtil::AppendText(const std::string& str, const std::string& out_filename)
{
	std::ofstream out_stream(out_filename, std::ios_base::app);

	bool succ = !out_stream.fail();
	if (succ)
	{
		out_stream << str;
	}

	out_stream.close();

	return succ;
}

std::string cFileUtil::ReadTextFile(FILE* f)
{
	if (!f)
	{
		return std::string("");
	}

	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);
	std::string text;
	std::unique_ptr<char> buffer(new char[size + 1]);

	buffer.get()[size] = 0;
	if (fread(buffer.get(), 1, size, f) == (unsigned long)size)
	{
		text = buffer.get();
	}

	buffer.reset();
	return text;
}
