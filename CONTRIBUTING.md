# Contributing to the Marine MFD Project

Thanks for your interest! Here are the guidelines for issues and pull requests.

## I found a bug

Before creating an issue:
1. **Search** existing issues to see if the problem has already been reported
2. **Check the "Known Issues" list** (issues with the `display-glitch` label)

Create a new issue with the following format:

**Title:** Short, concise description (max 60 characters)

**Description:**
- What happened?
- What did you expect?

**Reproduction:**
1. Go to screen X
2. Click button Y
3. ...

**Affected Hardware:**
- ESP32-4848S040 (original or modified?)

**Additional Info:**
- [ ] Photo / video of the problem
- [ ] Serial log output

## I have an idea for an improvement

Please use **GitHub Discussions** (category "Ideas"), not Issues.

## I want to contribute code

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Test your changes on real hardware
4. Create a pull request referencing the issue (`Fixes #123`)

## Known Issues (will not be fixed)

- **Horizontal display flicker** – See Issue #1. Fixing this requires deep modifications to the ESP32 RGB driver. Contributions welcome!