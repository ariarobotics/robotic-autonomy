# Chapter 00 – Introduction

> **Goal:** By the end of this chapter you should be familiar with the ROS2 fundemental concepts and their core commands.

---

## Objectives
By the end of this chapter you will be able to:
- Understand Nodes and its basics
- Understand Topics and its basics
- Understand Services and its basics
- Understand Parameters
- Understand Actions and its basics

---

## Prerequisites
- ROS 2 Humble installed ([README](../../README.md))  

---

## 1. Why this matters
All of the above topics mentioned in objectives make up what is ROS2. These topics are fundemental to using ROS2 in a powerful way and having a greater understanding of these will prove to be a useful tool in robotic applications. They are the building blocks of robotic software.

---

## 2. Nodes

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---
## 3. Topics

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---


## 3. Services

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---

## 4. Parameters
Parameters are exactly that. They're a parameter (or an argument for broader coding terms) that you can pass through with a node (or anything else). These are changable at run time and don't require any editting of the code.

1. **To display passable parameters for active nodes**  
   ```bash
   ros2 param list <node_name>
   ```
   _Node name is optional, but passing it will show all parameters that are specific to the node._

2. **To view all the values of a node's parameters**  
   ```bash
   ros2 param dump <node_name>
   ```
   _It is possible to "archive" or save the current settings of that node’s parameters by adding a `> <file_name>.yaml` after the node name. It will automatically make a file if needed._

3. **To load the presets from a `.yaml` file**  
   ```bash
   ros2 param load <node_name> <file_name>
   ```

4. **To find the type and the current value of a parameter**  
   ```bash
   ros2 param get <node_name> <parameter_name>
   ```

5. **To change the value of a parameter**  
   ```bash
   ros2 param set <node_name> <parameter_name> <value>
   ```
It should be noted that any changed to parameters are __not__ permanent. If you plan to use a set parameter over multiple sessions it's important to save and load them with a `.yaml` file.

---

## 5. Actions

1. **Nodes**  
   ```bash
   # code or command here
   ```
   _Explanation: what’s happening and why it matters._

2. **Topics**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

3. **Services**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

4. **Parameters**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

5. **Actions**  
   ```bash
   # next command or snippet
   ```
   _Notes/pitfalls to watch for._

---

## 4. Try it

> A brief exercise to reinforce the chapter—what to run or modify next.

```bash
# e.g. invoke a node, tweak a param, view output, etc.
```

---

## 5. Common errors & fixes (if any)

| Symptom                         | Likely cause                   | Quick fix                          |
|---------------------------------|--------------------------------|------------------------------------|
| `command not found`             | You forgot to `source` the setup script. | `source install/setup.bash`        |
| Node crashes on startup         | Bad parameter file or YAML indentation.   | Check syntax/indentation in your params file. |


---

## Further reading and references
- Official ROS 2 docs: [Title of section](https://docs.ros.org/en/humble/…)
- Any other link you have used and though is good.
