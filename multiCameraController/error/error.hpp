#pragma once

#include <iostream>
#include <sstream>
#include <windows.h>


class Error : public std::runtime_error {
public:
    Error(
        const std::string& type, 
        const std::string& msg, 
        int code
    ): 
        std::runtime_error("[Error] " + type + ": " + msg + " (code: " + std::to_string(code) + ")"),
        type_(type), 
        message_(msg), 
        code_(code) 
        {}

    const std::string& getType() const noexcept { return type_; }
    const std::string& getMessage() const noexcept { return message_; }
    int getCode() const noexcept { return code_; }

private:
    std::string type_;
    std::string message_;
    int code_;
};

/**
 * @class: MediaFoundation
 */
class MediaFoundationError: public Error {
public:
    MediaFoundationError(const std::string& msg, int code = 400)
        : Error("MediaFoundationError", msg, code) {}
};

/**
 * @class: Callback
 */
class CallbackError: public Error {
public:
    CallbackError(const std::string& msg, int code = 400)
        : Error("CallbackError", msg, code) {}
};

/**
 * @class: Sampler
 */
class SamplerError: public Error {
public:
    SamplerError(const std::string& msg, int code = 400)
        : Error("SamplerError", msg, code) {}
};

/**
 * @class: MediaMaker
 */
class MediaMakerError : public Error {
public:
    MediaMakerError(const std::string& msg, int code = 700)
        : Error("MediaMakerError", msg, code) {}
};

/**
 * @class: CSVMaker
 */
class CSVMakerError : public Error {
public:
    CSVMakerError(const std::string& msg, int code = 800)
        : Error("CSVMakerError", msg, code) {}
};

/**
 * @class: Manager
 */
class ManagerError : public Error {
public:
    ManagerError(const std::string& msg, int code = 900)
        : Error("ManagerError", msg, code) {}
};

/**
 * @class: MultiManager
 */
class MultiManagerError : public Error {
public:
    MultiManagerError(const std::string& msg, int code = 1000)
        : Error("MultiManagerError", msg, code) {}
};