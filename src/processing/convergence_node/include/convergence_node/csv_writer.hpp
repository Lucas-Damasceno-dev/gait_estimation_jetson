#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP

#include <string>
#include <vector>

struct CsvRow {
    std::vector<double> values;
};

class CsvWriter
{
public:
    CsvWriter() = default;
    void add_row(const CsvRow& row);
    void save(const std::string& filename, const std::vector<std::string>& header = {}) const;
    void clear();

private:
    std::vector<CsvRow> rows_;
};

#endif // CSV_WRITER_HPP