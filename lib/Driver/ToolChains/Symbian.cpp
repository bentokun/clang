//===--- Solaris.cpp - Solaris ToolChain Implementations --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "CommonArgs.h"
#include "Symbian.h"
#include "clang/Config/config.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/Options.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"

using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;
using namespace clang;
using namespace llvm::opt;

void symbian::Assembler::ConstructJob(Compilation &C, const JobAction &JA,
                                      const InputInfo &Output,
                                      const InputInfoList &Inputs,
                                      const ArgList &Args,
                                      const char *LinkingOutput) const {
  claimNoWarnArgs(Args);
  ArgStringList CmdArgs;

  Args.AddAllArgValues(CmdArgs, options::OPT_Wa_COMMA, options::OPT_Xassembler);

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  for (const auto &II : Inputs)
    CmdArgs.push_back(II.getFilename());

  const char *Exec = Args.MakeArgString(
      getToolChain().GetProgramPath("arm-none-symbianelf-as"));

  C.addCommand(llvm::make_unique<Command>(JA, *this, Exec, CmdArgs, Inputs));
}

void symbian::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                                   const InputInfo &Output,
                                   const InputInfoList &Inputs,
                                   const ArgList &Args,
                                   const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  Args.AddAllArgs(CmdArgs, options::OPT_L);
  Args.AddAllArgs(CmdArgs, options::OPT_T_Group);
  Args.AddAllArgs(CmdArgs, options::OPT_e);
  Args.AddAllArgs(CmdArgs, options::OPT_s);
  Args.AddAllArgs(CmdArgs, options::OPT_t);
  Args.AddAllArgs(CmdArgs, options::OPT_r);

  CmdArgs.push_back("-e_E32Startup");

  if (Output.isFilename()) {
    CmdArgs.push_back("-o");
    CmdArgs.push_back(Output.getFilename());
  } else {
    assert(Output.isNothing() && "Invalid output.");
  }

  if (!Args.hasArg(options::OPT_epoc_ignore_linker_lib_dir)) {
    if (llvm::Optional<std::string> epocLinkerLibDir =
            llvm::sys::Process::GetEnv("EPOCLIBSEARCHDIR")) {
      long int parsePointer = 0;
      long int lastMatch = 0;

      while (parsePointer < epocLinkerLibDir->length()) {
        if (epocLinkerLibDir->at(parsePointer++) == ';') {
          CmdArgs.push_back(
              Args.MakeArgString(std::string("-L") +
                                 epocLinkerLibDir->substr(
                                     lastMatch, parsePointer - lastMatch - 1)));

          lastMatch = parsePointer;
        }
      }

      CmdArgs.push_back(Args.MakeArgString(
          std::string("-L") +
          epocLinkerLibDir->substr(lastMatch, parsePointer - lastMatch + 1)));
    }
  }

  getToolChain().AddFilePathLibArgs(Args, CmdArgs);

  // The rest of the libraries requires developer to link correct libraries, let
  // them do it theirselves.
  AddLinkerInputs(getToolChain(), Inputs, Args, CmdArgs, JA);

  llvm::Triple osTriple = getToolChain().getTriple();

  const unsigned int epocVer = osTriple.getOSMajorVersion();

  switch (epocVer) {
  default:
  case 0:
  case 9:
  case 10: {
    CmdArgs.push_back(Args.MakeArgString("-lsupc++"));
    break;
  }
  }

  CmdArgs.push_back(Args.MakeArgString("-lgcc"));

  // Prevent unwind errors
  CmdArgs.push_back(Args.MakeArgString("-lgcc_eh"));

  // Link entry point (eexe.lib)
  if (!Args.hasArg(options::OPT_nostdlib, options::OPT_nostartfiles)) {
    CmdArgs.push_back(Args.MakeArgString("-l:euser.dso"));

    if (Args.hasArg(options::OPT_symbiancrt) &&
        !Args.hasArg(options::OPT_shared)) {
      // Link this with ecrt0.lib (POSIX)
      CmdArgs.push_back(Args.MakeArgString("-l:ecrt0.lib"));
    } else {
      if (Args.hasArg(options::OPT_shared)) {
        // This is a DLL, link it with edll.lib
        CmdArgs.push_back(Args.MakeArgString("-l:edll.lib"));
        CmdArgs.push_back(Args.MakeArgString("-l:edllstub.lib"));
      } else {
        CmdArgs.push_back(Args.MakeArgString("-l:eexe.lib"));
        CmdArgs.push_back(Args.MakeArgString("-l:usrt2_2.lib"));
      }
    }

    CmdArgs.push_back(Args.MakeArgString("-l:libc.dso"));
    CmdArgs.push_back(Args.MakeArgString("-l:libstdcpp.dso"));

    switch (osTriple.getEnvironment()) {
    default:
    case llvm::Triple::GNUEABI:
    case llvm::Triple::GNUEABIHF:
    case llvm::Triple::EABIHF:
    case llvm::Triple::MuslEABI:
    case llvm::Triple::MuslEABIHF: {
      // Link with EABI libraries
      CmdArgs.push_back(Args.MakeArgString("-l:drtaeabi.dso"));
      CmdArgs.push_back(Args.MakeArgString("-l:dfpaeabi.dso"));
      CmdArgs.push_back(Args.MakeArgString("-l:dfprvct2_2.dso"));
      CmdArgs.push_back(Args.MakeArgString("-l:drtrvct2_2.dso"));
      CmdArgs.push_back(Args.MakeArgString("-l:scppnwdl.dso"));

      break;
    }
    }
  }

  CmdArgs.push_back(Args.MakeArgString("--no-warn-mismatch"));

  std::string programPath =
      getToolChain().GetProgramPath("arm-none-symbianelf-ld");

  const char *Exec = Args.MakeArgString(programPath);
  C.addCommand(llvm::make_unique<Command>(JA, *this, Exec, CmdArgs, Inputs));
}

Symbian::Symbian(const Driver &D, const llvm::Triple &Triple,
                 const ArgList &Args)
    : Generic_ELF(D, Triple, Args) {
  GCCInstallation.init(Triple, Args);
  path_list &Paths = getFilePaths();

  if (Args.hasArg(options::OPT_epocroot)) {
    epocRoot = Args.getLastArgValue(options::OPT_epocroot);
  }

  if (!epocRoot) {
    // Try to get environment variable
    epocRoot = llvm::sys::Process::GetEnv("EPOCROOT");
  }

  std::string armPrefixEpoc = "armv5";

  if (epocRoot) {
    StringRef epocRootDir(*epocRoot);

    // Add path toghether
    SmallString<512> epocReleaseDir(epocRootDir);
    llvm::sys::path::append(epocReleaseDir, "epoc32/release/");

    // Add this path to the library path search
    addPathIfExists(D, epocReleaseDir, Paths);

    // Get the EPOC user libraries path
    SmallString<512> epocReleaseLibrariesDir(epocRootDir);
    llvm::sys::path::append(epocReleaseLibrariesDir,
                            std::string("epoc32/release/") + armPrefixEpoc +
                                "/lib/");

    SmallString<512> epocBootstrapLinkLibrariesDir(epocRootDir);
    llvm::sys::path::append(epocBootstrapLinkLibrariesDir,
                            std::string("epoc32/release/") + armPrefixEpoc +
                                "/udeb/");

    addPathIfExists(D, epocReleaseLibrariesDir, Paths);

    // Using debug libraries as default
    addPathIfExists(D, epocBootstrapLinkLibrariesDir, Paths);
  }
}

SanitizerMask Symbian::getSupportedSanitizers() const {
  SanitizerMask Res = ToolChain::getSupportedSanitizers();
  Res |= SanitizerKind::Vptr;

  return Res;
}

Tool *Symbian::buildAssembler() const {
  return new tools::symbian::Assembler(*this);
}

Tool *Symbian::buildLinker() const { return new tools::symbian::Linker(*this); }

void Symbian::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                        ArgStringList &CC1Args) const {
  const Driver &D = getDriver();

  if (DriverArgs.hasArg(clang::driver::options::OPT_nostdinc))
    return;

  // Add EPOCROOT include if it exists
  if (epocRoot) {
    SmallString<512> epocIncludeDir(*epocRoot);
    llvm::sys::path::append(epocIncludeDir, "epoc32/include/");

    addSystemInclude(DriverArgs, CC1Args, epocIncludeDir);

    if (!DriverArgs.hasArg(options::OPT_nostdlibinc)) {
      // As long as the appliciation is linked as CRT or initialized with
      // Symbian C++ we include stdapis in.
      SmallString<512> epocStdDir(*epocRoot);
      llvm::sys::path::append(epocStdDir, "epoc32/include/stdapis/");
      
      SmallString<512> epocStlPortDir(*epocRoot);
      llvm::sys::path::append(epocStlPortDir, "epoc32/include/stdapis/stlport/");

      SmallString<512> epocStlPortStlDir(*epocRoot);
      llvm::sys::path::append(epocStlPortDir,
                              "epoc32/include/stdapis/stlport/stl");

      addSystemInclude(DriverArgs, CC1Args, epocStdDir);
      addSystemInclude(DriverArgs, CC1Args, epocStlPortDir);
      addSystemInclude(DriverArgs, CC1Args, epocStlPortStlDir);
    }
  }

  if (!DriverArgs.hasArg(options::OPT_nobuiltininc)) {
    SmallString<128> P(D.ResourceDir);
    llvm::sys::path::append(P, "include");
    addSystemInclude(DriverArgs, CC1Args, P);
  }
}

void Symbian::addLibStdCxxIncludePaths(
    const llvm::opt::ArgList &DriverArgs,
    llvm::opt::ArgStringList &CC1Args) const {
}
