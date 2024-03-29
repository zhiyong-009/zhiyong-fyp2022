#ifndef RAM_PATH_PLANNING_ERROR_OBSERVER_HPP
#define RAM_PATH_PLANNING_ERROR_OBSERVER_HPP

#include <string>

#include <vtkCommand.h>

class ErrorObserver : public vtkCommand
{
public:
  ErrorObserver() :
          Error(false),
          Warning(false),
          ErrorMessage(""),
          WarningMessage("")
  {
  }

  /**
   * Returns a new instance on a ErrorObserver
   */
  static ErrorObserver *New()
  {
    return new ErrorObserver;
  }

  /** @brief Call this function to know either or not an error has occurred
   *  @return true if there is any error false otherwise */
  bool inline GetError() const
  {
    return Error;
  }

  /** @brief Call this function to know either or not a warning has occurred
   *  @return true if there is any warning false otherwise */
  bool inline GetWarning() const
  {
    return Warning;
  }

  /** @brief Clear all members of the object */
  void inline Clear()
  {
    Error = false;
    Warning = false;
    ErrorMessage = "";
    WarningMessage = "";
  }

  virtual void Execute(vtkObject *vtkNotUsed(caller),
                       unsigned long event,
                       void * calldata)
  {
    switch (event)
    {
      case vtkCommand::ErrorEvent:
        ErrorMessage += static_cast<char*>(calldata);
        Error = true;
        break;
      case vtkCommand::WarningEvent:
        WarningMessage += static_cast<char*>(calldata);
        Warning = true;
        break;
    }
  }

  /** @brief Call this function to get the error message if there is any
   *  @return a string containing the error message */
  inline std::string GetErrorMessage() const
  {
    return ErrorMessage;
  }

  /** @brief Call this function to get the warning message if there is any
   *  @return a string containing the warning message */
  inline std::string GetWarningMessage() const
  {
    return WarningMessage;
  }

private:
  bool Error;
  bool Warning;
  std::string ErrorMessage;
  std::string WarningMessage;
};

#endif

