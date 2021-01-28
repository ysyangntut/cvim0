#include "CvimProject.h"

#include <string>
#include <iostream>
#include <sstream>

#include <Target.h>

CvimProject::CvimProject()
{
    // data initialization
    this->fileSequences.clear();
}

// adds a new file sequence by file names in c-style format
// returns number of files added. Returns a value <= 0 if there
// is something wrong.
int CvimProject::addFileSequence(
        std::string name,       //!< name of the new file sequence
        std::string fnames,     //!< file names of the files, requires a %d as it is c-style format
        int         startIndex, //!< the starting index of the %d in fnames
        int         nFiles     //!< number of files
        )
{
    // This function only add new sequence rather than replacing one.
    // It checks if the name is a new name. If it is not, this function returns -1.
    if (this->fileSequences.find(name) != this->fileSequences.end()) {
        // If this name has been existed already, it returns -1.
        return -1;
    }

    // Add file names into vector<string>
    char buf[10000];
    std::vector<std::string> fileSequence(nFiles);
    for (int i = 0; i < nFiles; i++) {
        snprintf(buf, 10000, fnames.c_str(), i + startIndex);
        fileSequence[i] = std::string(buf);
    }

    // Add vector<string> (a file sequence) into file sequences
    this->fileSequences[name] = fileSequence;

    return nFiles;
}

//! \brief CvimProject::printFileSequenceInfo() prints the information
//! of all file sequences. Considering the numbers of files could be
//! huge, only some of them (maybe the first and the last) are displayed.
//! \return 0
int CvimProject::printFileSequenceInfo() const
{
    std::stringstream ss;
    std::string sstr;
    char buf[10000];

    // Number of file sequences
    snprintf(buf, 10000, "Number of File Sequences: %d\n", (int) this->fileSequences.size());
    ss << buf;

    // Names of file sequences followed by their number of files.
    for(auto const& imap: this->fileSequences) {
        snprintf(buf, 10000, "File Sequence name: [%s]\n", imap.first.c_str());
        ss << buf;
        snprintf(buf, 10000, "  Number of files: %d\n", (int) imap.second.size());
        ss << buf;
        snprintf(buf, 10000, "  First file: %s\n", imap.second[0].c_str());
        ss << buf;
        snprintf(buf, 10000, "  Last  file: %s\n", imap.second[imap.second.size() - 1].c_str());
        ss << buf;
    }
    std::cout << ss.str();
    return 0;
}

//! \brief CvimProject::printFileSequenceInfo(nameImgSeq) prints the
//! informationof the specified file sequence. Considering the numbers
//! of files could be huge, only some of them (maybe the first and the
//! last) are displayed.
//! \return 0
int CvimProject::printFileSequenceInfo(std::string nameImgSeq) const
{
    std::stringstream ss;
    std::string sstr;
    char buf[10000];

    auto const& imap = this->fileSequences.find(nameImgSeq);
    if (imap == this->fileSequences.end()) {
        ss << "File Sequence name: [" << nameImgSeq << "] does not exist.\n";
        std::cout << ss.str();
        return -1;
    } else {
        snprintf(buf, 10000, "File Sequence name: [%s]\n", imap->first.c_str());
        ss << buf;
        snprintf(buf, 10000, "  Number of files: %d\n", (int) imap->second.size());
        ss << buf;
        snprintf(buf, 10000, "  First file: %s\n", imap->second[0].c_str());
        ss << buf;
        snprintf(buf, 10000, "  Last file: %s\n", imap->second[imap->second.size() - 1].c_str());
        ss << buf;
    }
    std::cout << ss.str();
    return 0;
}

//! \brief CvimProject::getFileSequence() returns the file sequence
//! in format of vector<string>.
//! \return the file sequence
const std::vector<std::string> & CvimProject::getFileSequence(std::string name) const
{
    static std::vector<std::string> theEmpty;
    auto const& imap = this->fileSequences.find(name);
    if (imap == this->fileSequences.end()) {
        return theEmpty;
    }
    return imap->second;
}

//! \brief CvimProject::getFileSequence() returns the file sequence
//! in format of vector<string>.
//! \return the file sequence
std::vector<std::string> & CvimProject::getFileSequence(std::string name)
{
    static std::vector<std::string> theEmpty;
    auto const& imap = this->fileSequences.find(name);
    if (imap == this->fileSequences.end()) {
        return theEmpty;
    }
    return imap->second;
}


void CvimProject::testMain()
{
    CvimProject theProject;

    // Add a sequence of file
    theProject.addFileSequence(
                "RotatingDonut",
                "D:/yuansen/Lectures/CVIM/projects/rotatingDonut/RollingDonut_%06d.JPG",
                0,
                132);

    theProject.printFileSequenceInfo();
    theProject.printFileSequenceInfo("RotatingDonut");
    int nDonutFile = (int) theProject.getFileSequence("RotatingDonut").size();
    std::cout << "There are " << nDonutFile << " files in RotatingDonut.\n";

    cv::imshow("TEST", cv::imread(theProject.getFileSequence("RotatingDonut")[0]));
    cv::waitKey(500);
//    theProject.getFileSequence("RotatingDonut")[0] = "XXXXXXXXXXXXXX";
//    theProject.printFileSequenceInfo("RotatingDonut");
//    cv::Mat theDonutFile = theProject.getFileFromFileSequence("RotatingDonut", 1);

}



