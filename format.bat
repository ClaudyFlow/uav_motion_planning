@echo off

for /r "%~dp0inc" %%f in (*.h *.hh *.hpp *.hxx *.c *.cc *.cpp *.cxx) do clang-format -i --style=Google --verbose "%%f"
for /r "%~dp0src" %%f in (*.h *.hh *.hpp *.hxx *.c *.cc *.cpp *.cxx) do clang-format -i --style=Google --verbose "%%f"
