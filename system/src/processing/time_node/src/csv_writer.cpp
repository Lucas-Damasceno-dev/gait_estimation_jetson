#include "csv_writer.hpp"
#include <fstream>

void CsvWriter::add_row(const CsvRow& row)
{
    rows_.push_back(row);
}

void CsvWriter::clear()
{
    rows_.clear();
}

void CsvWriter::save(const std::string& filename, const std::vector<std::string>& header) const
{
    std::ofstream file(filename);
    if (!file.is_open()) return;

    if (!header.empty()) {
        for (size_t i = 0; i < header.size(); i++) {
            file << header[i];
            if (i + 1 < header.size()) file << ",";
        }
        file << "\n";
    }

    // escreve as linhas
    for (const auto& row : rows_) {
        for (size_t i = 0; i < row.values.size(); i++) {
            file << row.values[i];
            if (i + 1 < row.values.size()) file << ",";
        }
        file << "\n";
    }

    file.close();
}