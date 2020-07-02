# Bolder Flight Systems Contributing Guide
The following guidelines set best practices for contributing to Bolder Flight Systems software repositories.

## Steps
Software development and contribution should follow these steps:
   * Develop
   * Test 
   * Document 
   * Merge
   * Tag

Further detail is provided below for each step.

### Develop
All development should occur on a separate branch from master. Software should be developed following our [Style Guide](#style). 

#### Style Guide<a name="style"></a>
Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Any parameter or variable names, which contain unit specific data should be appended with an underscore and the units (i.e. accel_z_mps2). Units should be named by their common abbreviation. For derived units, use _p_ to indicate _"per"_ and append exponents. So, m/s/s would be _mps2_ and kg/m^3 would be _kgpm3_. If a common abbreviation already exists for the unit, use that instead (i.e. _psi_ instead of _lbpin2_).

Prefer to use _float_ instead of _double_ unless there is a specific need for improved accuracy. Specify integer size in all cases where a certain number of bytes are expected (i.e. uint16_t where 2 bytes is assumed); otherwise use unsigned or signed int. Typically, we are not concerned with program size and unsigned or signed int work well for integers and index values. If a certain number of bytes are needed they need to be called out directly since different compilers and platforms can change the number of bytes stored in a short, for instance. Do not assume that using unsigned int will prevent a negative input, instead use signed int and check for negative values (see the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for more details on why).

#### External sources licensing
If you use external sources, ensure they are licensed MIT, BSD, or a similarly permissive license. We would like to limit the amount of LGPL code in our code base and need to avoid all GPL and unlicensed code.

If you are using external sources with licenses other than MIT or BSD, contact [Brian](mailto:brian.taylor@bolderflight.com) to discuss options.

#### Examples
Develop examples demonstrating your code's functionality and include expected outputs in comments. These examples provide an easy access point to learning your code and ensuring that it installed correctly.

### Tests
All tests should pass before a pull request is issued. At a minimum, the following tests should be run:
   * Linting
   * Build
   * Test
      * Inputs
      * Expected values

Tests are run using the [Google Test framework](https://github.com/google/googletest).
#### Linting
Linting tests check for conformance to the style guide - analyzing the code for potential errors and leading to better readibility. [cpplint](https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py) should be used to conduct linting tests with verbosity level 0.

#### Build
Libraries, example code, and tests should be compiled with CMake without error. We typically use CMake with an AMD64 Linux target for general libraries. CMake should be [built from source](https://github.com/Kitware/CMake) and gcc compilers available on the target system. 

#### Inputs
Test all parameters against unexpected values, such as NULL inputs, buffer overflows, and zero or negative values. Try to capture all potential combinations of malformed inputs to ensure that your code is protecting against these. Assume that your fellow developers will not read your documentation and will try to use your code with incorrect parameters.

#### Expected values
Test against expected values to ensure that your software algorithms are computing outputs correctly.

#### CI Pipeline
Update the CI pipeline configuration in _.gitlab-ci.yml_ to incorporate all additional tests that you add. The _bfs_ tag should be used to specify using Bolder Flight Systems' runners, which are configured to compile and test our software.

### Document
Document all API changes in the repository README.md file. Specify what each function and method does, input parameters, and outputs. Give a short example of how to use that block of code. Ideally these example snippets will be pulled from the example executable.

### Merge
Pushing directly to the master branch is dangerous - it enables changes to be made without proper testing and review. This practice also increases the risk of pushing breaking changes and having inadequate documentation. You should create a branch, make your changes, run tests, update documentation, and submit a merge request for review and incorporation.

The CHANGELOG.md will be updated to briefly document code changes.

### Tag
Tags are used to specify release version numbers in [semver](https://semver.org/) formatting. These tags are important because repositories using your code as a dependency will pull, build, and validate against a specific version, rather than continuously needing to manage code updates following HEAD.
