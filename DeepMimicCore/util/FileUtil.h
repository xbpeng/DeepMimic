#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"

class cFileUtil
{
public:
	static FILE* OpenFile(const std::string& file_name, const char* mode);
	static FILE* OpenFile(const char* file_name, const char* mode);
	static void CloseFile(FILE*& f);
	static void ClearFile(const std::string& file_name);
	static void CreateFile(const std::string& file_name);
	static void DeleteFile(const char* file_name);
	static std::string RemoveExtension(const std::string& filename);
	static void DeleteFile(const std::string& filename);
	static long int GetFileSize(const std::string& filename);
	static std::string GetExtension(const std::string& filename);
	static std::string GetFilename(const std::string& path);
	static void FilterFilesByExtension(std::vector<std::string>& files, const std::string& ext);
	static bool ExistsFile(const std::string& file_name);

	static void FindLine(std::ifstream& f_stream, int line);
	static std::string ReadTextFile(const std::string& path);

	// static bool ReadArray(FILE* f, const std::string& tag_beg, const std::string& tag_end, std::vector<double>& out_buffer);
	static bool ReadTable(const std::string& filename, std::vector<std::vector<double>>& out_buffer);
	static bool ReadMatrix(const std::string& filename, Eigen::MatrixXd& out_mat);
	static bool WriteMatrix(const Eigen::MatrixXd& mat, const std::string& out_filename);

	static bool AppendText(const std::string& str, const std::string& out_filename);

private:
	static std::string ReadTextFile(FILE* f);
};
