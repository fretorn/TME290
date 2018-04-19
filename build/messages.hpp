
/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef MYTESTMESSAGE1_HPP
#define MYTESTMESSAGE1_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>

using namespace std::string_literals; // NOLINT
class LIB_API MyTestMessage1 {
    public:
        MyTestMessage1() = default;
        MyTestMessage1(const MyTestMessage1&) = default;
        MyTestMessage1& operator=(const MyTestMessage1&) = default;
        MyTestMessage1(MyTestMessage1&&) noexcept = default; // NOLINT
        MyTestMessage1& operator=(MyTestMessage1&&) noexcept = default; // NOLINT
        ~MyTestMessage1() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        MyTestMessage1& myValue(const uint16_t &v) noexcept;
        uint16_t myValue() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint16_t"s), std::move("myValue"s), m_myValue, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint16_t"s), std::move("myValue"s), m_myValue, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint16_t m_myValue{ 0 }; // field identifier = 1.
        
};


template<>
struct isVisitable<MyTestMessage1> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<MyTestMessage1> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef MYTESTMESSAGE2_HPP
#define MYTESTMESSAGE2_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>

using namespace std::string_literals; // NOLINT
class LIB_API MyTestMessage2 {
    public:
        MyTestMessage2() = default;
        MyTestMessage2(const MyTestMessage2&) = default;
        MyTestMessage2& operator=(const MyTestMessage2&) = default;
        MyTestMessage2(MyTestMessage2&&) noexcept = default; // NOLINT
        MyTestMessage2& operator=(MyTestMessage2&&) noexcept = default; // NOLINT
        ~MyTestMessage2() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        MyTestMessage2& myValue1(const bool &v) noexcept;
        bool myValue1() const noexcept;
        
        MyTestMessage2& myValue2(const uint8_t &v) noexcept;
        uint8_t myValue2() const noexcept;
        
        MyTestMessage2& myValue3(const int8_t &v) noexcept;
        int8_t myValue3() const noexcept;
        
        MyTestMessage2& myValue4(const uint16_t &v) noexcept;
        uint16_t myValue4() const noexcept;
        
        MyTestMessage2& myValue5(const int16_t &v) noexcept;
        int16_t myValue5() const noexcept;
        
        MyTestMessage2& myValue6(const uint32_t &v) noexcept;
        uint32_t myValue6() const noexcept;
        
        MyTestMessage2& myValue7(const int32_t &v) noexcept;
        int32_t myValue7() const noexcept;
        
        MyTestMessage2& myValue8(const uint64_t &v) noexcept;
        uint64_t myValue8() const noexcept;
        
        MyTestMessage2& myValue9(const int64_t &v) noexcept;
        int64_t myValue9() const noexcept;
        
        MyTestMessage2& myValue10(const float &v) noexcept;
        float myValue10() const noexcept;
        
        MyTestMessage2& myValue11(const double &v) noexcept;
        double myValue11() const noexcept;
        
        MyTestMessage2& myValue12(const std::string &v) noexcept;
        std::string myValue12() const noexcept;
        
        MyTestMessage2& myValue13(const MyTestMessage1 &v) noexcept;
        MyTestMessage1 myValue13() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("bool"s), std::move("myValue1"s), m_myValue1, visitor);
            
            doVisit(2, std::move("uint8_t"s), std::move("myValue2"s), m_myValue2, visitor);
            
            doVisit(3, std::move("int8_t"s), std::move("myValue3"s), m_myValue3, visitor);
            
            doVisit(4, std::move("uint16_t"s), std::move("myValue4"s), m_myValue4, visitor);
            
            doVisit(5, std::move("int16_t"s), std::move("myValue5"s), m_myValue5, visitor);
            
            doVisit(6, std::move("uint32_t"s), std::move("myValue6"s), m_myValue6, visitor);
            
            doVisit(7, std::move("int32_t"s), std::move("myValue7"s), m_myValue7, visitor);
            
            doVisit(8, std::move("uint64_t"s), std::move("myValue8"s), m_myValue8, visitor);
            
            doVisit(9, std::move("int64_t"s), std::move("myValue9"s), m_myValue9, visitor);
            
            doVisit(10, std::move("float"s), std::move("myValue10"s), m_myValue10, visitor);
            
            doVisit(11, std::move("double"s), std::move("myValue11"s), m_myValue11, visitor);
            
            doVisit(12, std::move("std::string"s), std::move("myValue12"s), m_myValue12, visitor);
            
            doVisit(13, std::move("MyTestMessage1"s), std::move("myValue13"s), m_myValue13, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("bool"s), std::move("myValue1"s), m_myValue1, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint8_t"s), std::move("myValue2"s), m_myValue2, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("int8_t"s), std::move("myValue3"s), m_myValue3, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("uint16_t"s), std::move("myValue4"s), m_myValue4, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("int16_t"s), std::move("myValue5"s), m_myValue5, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("uint32_t"s), std::move("myValue6"s), m_myValue6, preVisit, visit, postVisit);
            
            doTripletForwardVisit(7, std::move("int32_t"s), std::move("myValue7"s), m_myValue7, preVisit, visit, postVisit);
            
            doTripletForwardVisit(8, std::move("uint64_t"s), std::move("myValue8"s), m_myValue8, preVisit, visit, postVisit);
            
            doTripletForwardVisit(9, std::move("int64_t"s), std::move("myValue9"s), m_myValue9, preVisit, visit, postVisit);
            
            doTripletForwardVisit(10, std::move("float"s), std::move("myValue10"s), m_myValue10, preVisit, visit, postVisit);
            
            doTripletForwardVisit(11, std::move("double"s), std::move("myValue11"s), m_myValue11, preVisit, visit, postVisit);
            
            doTripletForwardVisit(12, std::move("std::string"s), std::move("myValue12"s), m_myValue12, preVisit, visit, postVisit);
            
            doTripletForwardVisit(13, std::move("MyTestMessage1"s), std::move("myValue13"s), m_myValue13, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        bool m_myValue1{ false }; // field identifier = 1.
        
        uint8_t m_myValue2{ 0 }; // field identifier = 2.
        
        int8_t m_myValue3{ 0 }; // field identifier = 3.
        
        uint16_t m_myValue4{ 0 }; // field identifier = 4.
        
        int16_t m_myValue5{ 0 }; // field identifier = 5.
        
        uint32_t m_myValue6{ 0 }; // field identifier = 6.
        
        int32_t m_myValue7{ 0 }; // field identifier = 7.
        
        uint64_t m_myValue8{ 0 }; // field identifier = 8.
        
        int64_t m_myValue9{ 0 }; // field identifier = 9.
        
        float m_myValue10{ 0.0f }; // field identifier = 10.
        
        double m_myValue11{ 0.0 }; // field identifier = 11.
        
        std::string m_myValue12{ ""s }; // field identifier = 12.
        
        MyTestMessage1 m_myValue13{  }; // field identifier = 13.
        
};


template<>
struct isVisitable<MyTestMessage2> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<MyTestMessage2> {
    static const bool value = true;
};
#endif

