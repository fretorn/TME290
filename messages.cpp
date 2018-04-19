#include <messages.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */


int32_t MyTestMessage1::ID() {
    return 2001;
}

const std::string MyTestMessage1::ShortName() {
    return "MyTestMessage1";
}
const std::string MyTestMessage1::LongName() {
    return "MyTestMessage1";
}

MyTestMessage1& MyTestMessage1::myValue(const uint16_t &v) noexcept {
    m_myValue = v;
    return *this;
}
uint16_t MyTestMessage1::myValue() const noexcept {
    return m_myValue;
}



#include <messages.hpp>

/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */


int32_t MyTestMessage2::ID() {
    return 2002;
}

const std::string MyTestMessage2::ShortName() {
    return "MyTestMessage2";
}
const std::string MyTestMessage2::LongName() {
    return "MyTestMessage2";
}

MyTestMessage2& MyTestMessage2::myValue1(const bool &v) noexcept {
    m_myValue1 = v;
    return *this;
}
bool MyTestMessage2::myValue1() const noexcept {
    return m_myValue1;
}

MyTestMessage2& MyTestMessage2::myValue2(const uint8_t &v) noexcept {
    m_myValue2 = v;
    return *this;
}
uint8_t MyTestMessage2::myValue2() const noexcept {
    return m_myValue2;
}

MyTestMessage2& MyTestMessage2::myValue3(const int8_t &v) noexcept {
    m_myValue3 = v;
    return *this;
}
int8_t MyTestMessage2::myValue3() const noexcept {
    return m_myValue3;
}

MyTestMessage2& MyTestMessage2::myValue4(const uint16_t &v) noexcept {
    m_myValue4 = v;
    return *this;
}
uint16_t MyTestMessage2::myValue4() const noexcept {
    return m_myValue4;
}

MyTestMessage2& MyTestMessage2::myValue5(const int16_t &v) noexcept {
    m_myValue5 = v;
    return *this;
}
int16_t MyTestMessage2::myValue5() const noexcept {
    return m_myValue5;
}

MyTestMessage2& MyTestMessage2::myValue6(const uint32_t &v) noexcept {
    m_myValue6 = v;
    return *this;
}
uint32_t MyTestMessage2::myValue6() const noexcept {
    return m_myValue6;
}

MyTestMessage2& MyTestMessage2::myValue7(const int32_t &v) noexcept {
    m_myValue7 = v;
    return *this;
}
int32_t MyTestMessage2::myValue7() const noexcept {
    return m_myValue7;
}

MyTestMessage2& MyTestMessage2::myValue8(const uint64_t &v) noexcept {
    m_myValue8 = v;
    return *this;
}
uint64_t MyTestMessage2::myValue8() const noexcept {
    return m_myValue8;
}

MyTestMessage2& MyTestMessage2::myValue9(const int64_t &v) noexcept {
    m_myValue9 = v;
    return *this;
}
int64_t MyTestMessage2::myValue9() const noexcept {
    return m_myValue9;
}

MyTestMessage2& MyTestMessage2::myValue10(const float &v) noexcept {
    m_myValue10 = v;
    return *this;
}
float MyTestMessage2::myValue10() const noexcept {
    return m_myValue10;
}

MyTestMessage2& MyTestMessage2::myValue11(const double &v) noexcept {
    m_myValue11 = v;
    return *this;
}
double MyTestMessage2::myValue11() const noexcept {
    return m_myValue11;
}

MyTestMessage2& MyTestMessage2::myValue12(const std::string &v) noexcept {
    m_myValue12 = v;
    return *this;
}
std::string MyTestMessage2::myValue12() const noexcept {
    return m_myValue12;
}

MyTestMessage2& MyTestMessage2::myValue13(const MyTestMessage1 &v) noexcept {
    m_myValue13 = v;
    return *this;
}
MyTestMessage1 MyTestMessage2::myValue13() const noexcept {
    return m_myValue13;
}



