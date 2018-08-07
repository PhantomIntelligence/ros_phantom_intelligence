#parse("C File Header.h")
#[[#ifndef]]# ${INCLUDE_GUARD}
#[[#define]]# ${INCLUDE_GUARD}

namespace {

class ${NAME} {
    protected:
    
    public:
    
        explicit ${NAME}();
        
        ~${NAME}() noexcept;
        
        ${NAME}(${NAME} const& other);
        
        ${NAME}(${NAME}&& other) noexcept;
        
        ${NAME}& operator=(${NAME} const& other)&;
        
        ${NAME}& operator=(${NAME}&& other)& noexcept;
        
        void swap(${NAME}& current, ${NAME}& other) noexcept;
};

}

#[[#endif]]# //${INCLUDE_GUARD}
