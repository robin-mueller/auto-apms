<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.8">
  <compound kind="class">
    <name>auto_apms_util::ActionClientWrapper</name>
    <filename>classauto__apms__util_1_1ActionClientWrapper.html</filename>
    <templarg>typename ActionT</templarg>
    <member kind="function">
      <type></type>
      <name>ActionClientWrapper</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>ad935c14401246dec3a6ee469ae470024</anchor>
      <arglist>(rclcpp::Node::SharedPtr node_ptr, const std::string &amp;action_name)</arglist>
    </member>
    <member kind="function">
      <type>ResultFuture</type>
      <name>syncSendGoal</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>ab9c78c4b5c9a591cb55ad66268248dd9</anchor>
      <arglist>(const Goal &amp;goal=Goal{}, const SendGoalOptions &amp;options=SendGoalOptions{}, const std::chrono::seconds server_timeout=std::chrono::seconds{3}, const std::chrono::seconds response_timeout=std::chrono::seconds{3})</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>syncCancelLastGoal</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>ad10df0fae286205fef13c6018dd6faa4</anchor>
      <arglist>(const std::chrono::seconds response_timeout=std::chrono::seconds{3})</arglist>
    </member>
    <member kind="function">
      <type>ClientGoalHandle::SharedPtr</type>
      <name>getActiveGoalHandle</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>a4cd242449282bd03372cd05c353548c7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; const Feedback &gt;</type>
      <name>getFeedback</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>aceb07ad7bcbdf433dfcd81b0f98f71cb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static ActionGoalStatus</type>
      <name>getGoalStatus</name>
      <anchorfile>classauto__apms__util_1_1ActionClientWrapper.html</anchorfile>
      <anchor>aa64c5038ccbee5b4c46d208a277491c5</anchor>
      <arglist>(const ResultFuture &amp;future)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>auto_apms_util::ActionContext</name>
    <filename>classauto__apms__util_1_1ActionContext.html</filename>
    <templarg>typename ActionT</templarg>
    <member kind="function">
      <type></type>
      <name>ActionContext</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a9edfc4cea03812930f0c7c8244881eff</anchor>
      <arglist>(rclcpp::Logger logger)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setUp</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a9c71f1e9239249ec73a85b21053c389a</anchor>
      <arglist>(std::shared_ptr&lt; GoalHandle &gt; goal_handle_ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publishFeedback</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a21af39d33e40a0e5731935e3119eabbc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>succeed</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>ad70fe2a2bc1f6f500fecb4fd7ee611a5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cancel</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>ac14ec591f8d26d1db57bfe7bf4e265c9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>abort</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>ae3c2f494f06a9c21053c4c93b3f15d1c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>invalidate</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a4f7934012e108331e46f56174b02d600</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isValid</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a34e59185020d5843c5537c717c6f48ab</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; GoalHandle &gt;</type>
      <name>getGoalHandlePtr</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a5e42892f08f8a292f0f3d865cd38177a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Feedback &gt;</type>
      <name>getFeedbackPtr</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a07c75ac76a98db544ef61c8d5581a311</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>std::shared_ptr&lt; Result &gt;</type>
      <name>getResultPtr</name>
      <anchorfile>classauto__apms__util_1_1ActionContext.html</anchorfile>
      <anchor>a443c8d14945573f72f53554f90f5beb4</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>auto_apms_util::ActionWrapper</name>
    <filename>classauto__apms__util_1_1ActionWrapper.html</filename>
    <templarg>typename ActionT</templarg>
    <member kind="function">
      <type></type>
      <name>ActionWrapper</name>
      <anchorfile>classauto__apms__util_1_1ActionWrapper.html</anchorfile>
      <anchor>a26096bf30c9cda890d28998bba0fcf6d</anchor>
      <arglist>(const std::string &amp;action_name, rclcpp::Node::SharedPtr node_ptr, std::shared_ptr&lt; ActionContextType &gt; action_context_ptr)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ActionWrapper</name>
      <anchorfile>classauto__apms__util_1_1ActionWrapper.html</anchorfile>
      <anchor>a67b6d6851121d84c4b492fd90db68624</anchor>
      <arglist>(const std::string &amp;action_name, rclcpp::Node::SharedPtr node_ptr)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ActionWrapper</name>
      <anchorfile>classauto__apms__util_1_1ActionWrapper.html</anchorfile>
      <anchor>afd36297c2d03c81ac0d225779e50daf2</anchor>
      <arglist>(const std::string &amp;action_name, const rclcpp::NodeOptions &amp;options)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>auto_apms_util::exceptions::ExceptionBase</name>
    <filename>classauto__apms__util_1_1exceptions_1_1ExceptionBase.html</filename>
  </compound>
  <compound kind="class">
    <name>auto_apms_util::PluginClassLoader</name>
    <filename>classauto__apms__util_1_1PluginClassLoader.html</filename>
    <templarg>typename BaseT</templarg>
    <member kind="function">
      <type></type>
      <name>PluginClassLoader</name>
      <anchorfile>classauto__apms__util_1_1PluginClassLoader.html</anchorfile>
      <anchor>a28adc5771b02b975fb40aa749fbe0370</anchor>
      <arglist>(const std::string &amp;base_package, const std::string &amp;base_class, const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::map&lt; std::string, std::string &gt;</type>
      <name>getClassPackageMap</name>
      <anchorfile>classauto__apms__util_1_1PluginClassLoader.html</anchorfile>
      <anchor>a3241d8dd420aff5a50adcb2a7b7831df</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static PluginClassLoader</type>
      <name>makeUnambiguousPluginClassLoader</name>
      <anchorfile>classauto__apms__util_1_1PluginClassLoader.html</anchorfile>
      <anchor>af06e811fe2b66ac0c13cfff236a30503</anchor>
      <arglist>(const std::string &amp;base_package, const std::string &amp;base_class, const std::set&lt; std::string &gt; &amp;exclude_packages={}, const std::map&lt; std::string, std::string &gt; &amp;reserved_names={})</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>auto_apms_util::exceptions::ResourceError</name>
    <filename>structauto__apms__util_1_1exceptions_1_1ResourceError.html</filename>
    <base>auto_apms_util::exceptions::ExceptionBase</base>
  </compound>
  <compound kind="struct">
    <name>auto_apms_util::exceptions::ResourceIdentityFormatError</name>
    <filename>structauto__apms__util_1_1exceptions_1_1ResourceIdentityFormatError.html</filename>
    <base>auto_apms_util::exceptions::ExceptionBase</base>
  </compound>
  <compound kind="struct">
    <name>auto_apms_util::exceptions::SetLoggingSeverityError</name>
    <filename>structauto__apms__util_1_1exceptions_1_1SetLoggingSeverityError.html</filename>
    <base>auto_apms_util::exceptions::ExceptionBase</base>
  </compound>
  <compound kind="struct">
    <name>auto_apms_util::exceptions::YAMLFormatError</name>
    <filename>structauto__apms__util_1_1exceptions_1_1YAMLFormatError.html</filename>
    <base>auto_apms_util::exceptions::ExceptionBase</base>
  </compound>
  <compound kind="namespace">
    <name>auto_apms_util</name>
    <filename>namespaceauto__apms__util.html</filename>
    <class kind="class">auto_apms_util::ActionClientWrapper</class>
    <class kind="class">auto_apms_util::ActionContext</class>
    <class kind="class">auto_apms_util::ActionWrapper</class>
    <class kind="class">auto_apms_util::PluginClassLoader</class>
    <member kind="enumeration">
      <type></type>
      <name>ActionGoalStatus</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8def3da5db948bd2e7727bc6bb2032cb</anchor>
      <arglist></arglist>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba9cc8ad3d99798c6726d4af0bd14e49d2">REJECTED</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba43491564ebcfd38568918efbd6e840fd">RUNNING</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba8f7afecbc8fbc4cd0f50a57d1172482e">COMPLETED</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>ActionStatus</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8bba9377700101e4a63f550e436d1ee1</anchor>
      <arglist></arglist>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1a43491564ebcfd38568918efbd6e840fd">RUNNING</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1ad0749aaba8b833466dfcbb0428e4f89c">SUCCESS</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1a36fc6065a3e970bc3e6b2e59da52bf2a">FAILURE</enumvalue>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>contains</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga26d1cc4c19097b3deb2cc7c98b25748c</anchor>
      <arglist>(const ContainerT&lt; ValueT, AllocatorT &gt; &amp;c, const ValueT &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; KeyT, CompareT, AllocatorT &gt;</type>
      <name>getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8979d95201a925f6524f9413e74b74f8</anchor>
      <arglist>(std::set&lt; KeyT, CompareT, AllocatorT &gt; c1, std::set&lt; KeyT, CompareT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; KeyT, AllocatorT &gt;</type>
      <name>getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1e57be4a3189f25e8c145aa93550c07d</anchor>
      <arglist>(std::vector&lt; KeyT, AllocatorT &gt; c1, std::vector&lt; KeyT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isFileEmpty</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga52a132b3de3720d7421ce7f49f7b766d</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>exposeToGlobalDebugLogging</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8b5c7b5382f9835da0e937d08a3ae66b</anchor>
      <arglist>(const rclcpp::Logger &amp;logger)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga7989c884b003bb1510022b4a1df41bd4</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, const std::string &amp;severity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga9e82bc25ea431ee055ef4728ae2d3e9c</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, rclcpp::Logger::Level severity)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>getPackagesWithResourceType</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1aa65bc57e6001b9a6254280bade88ef</anchor>
      <arglist>(const std::string &amp;resource_type, const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>getPackagesWithPluginResources</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gabbaa3e1d45d5733e8348af54b61841fa</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getPluginXMLPath</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga5895aab44d575c01e18de138c9ad0992</anchor>
      <arglist>(const std::string &amp;package)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>collectPluginXMLPaths</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8aef66ef74d7e817b3dba660a2ab3bbe</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>splitString</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga5c77f49068aaa8b8a7f9f4c039359f99</anchor>
      <arglist>(const std::string &amp;str, const std::string &amp;delimiter, bool remove_empty=true)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>printMap</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gae1b8af60a949b81dce119c4f260bc773</anchor>
      <arglist>(const std::map&lt; std::string, std::string &gt; &amp;map, const std::string &amp;key_val_sep=&quot;=&quot;, const std::string &amp;entry_sep=&quot;, &quot;)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>makeColoredText</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga0fa1ba55c5e0a44a705403ee5c1fb8cf</anchor>
      <arglist>(const std::string &amp;text, TextColor color)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>trimWhitespaces</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga297493100cc10a04f7568ec38b26b924</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>toCamelCase</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gaff05f09c20fe90c8c2c605da52e20a22</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>toSnakeCase</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga3e7540f3fcaffba2d61838d39e5f5713</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>contains</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga26d1cc4c19097b3deb2cc7c98b25748c</anchor>
      <arglist>(const ContainerT&lt; ValueT, AllocatorT &gt; &amp;c, const ValueT &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; KeyT, CompareT, AllocatorT &gt;</type>
      <name>getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8979d95201a925f6524f9413e74b74f8</anchor>
      <arglist>(std::set&lt; KeyT, CompareT, AllocatorT &gt; c1, std::set&lt; KeyT, CompareT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; KeyT, AllocatorT &gt;</type>
      <name>getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1e57be4a3189f25e8c145aa93550c07d</anchor>
      <arglist>(std::vector&lt; KeyT, AllocatorT &gt; c1, std::vector&lt; KeyT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isFileEmpty</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga52a132b3de3720d7421ce7f49f7b766d</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>exposeToGlobalDebugLogging</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8b5c7b5382f9835da0e937d08a3ae66b</anchor>
      <arglist>(const rclcpp::Logger &amp;logger)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga7989c884b003bb1510022b4a1df41bd4</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, const std::string &amp;severity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga9e82bc25ea431ee055ef4728ae2d3e9c</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, rclcpp::Logger::Level severity)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>getPackagesWithResourceType</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1aa65bc57e6001b9a6254280bade88ef</anchor>
      <arglist>(const std::string &amp;resource_type, const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>getPackagesWithPluginResources</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gabbaa3e1d45d5733e8348af54b61841fa</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>getPluginXMLPath</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga5895aab44d575c01e18de138c9ad0992</anchor>
      <arglist>(const std::string &amp;package)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>collectPluginXMLPaths</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8aef66ef74d7e817b3dba660a2ab3bbe</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>auto_apms_util</name>
    <title>Common Utilities</title>
    <filename>group__auto__apms__util.html</filename>
    <namespace>auto_apms_util</namespace>
    <class kind="class">auto_apms_util::ActionClientWrapper</class>
    <class kind="class">auto_apms_util::ActionContext</class>
    <class kind="class">auto_apms_util::ActionWrapper</class>
    <class kind="class">auto_apms_util::PluginClassLoader</class>
    <member kind="enumeration">
      <type></type>
      <name>auto_apms_util::ActionGoalStatus</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8def3da5db948bd2e7727bc6bb2032cb</anchor>
      <arglist></arglist>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba9cc8ad3d99798c6726d4af0bd14e49d2">REJECTED</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba43491564ebcfd38568918efbd6e840fd">RUNNING</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8def3da5db948bd2e7727bc6bb2032cba8f7afecbc8fbc4cd0f50a57d1172482e">COMPLETED</enumvalue>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>auto_apms_util::ActionStatus</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8bba9377700101e4a63f550e436d1ee1</anchor>
      <arglist></arglist>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1a43491564ebcfd38568918efbd6e840fd">RUNNING</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1ad0749aaba8b833466dfcbb0428e4f89c">SUCCESS</enumvalue>
      <enumvalue file="group__auto__apms__util.html" anchor="gga8bba9377700101e4a63f550e436d1ee1a36fc6065a3e970bc3e6b2e59da52bf2a">FAILURE</enumvalue>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::makeColoredText</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga0fa1ba55c5e0a44a705403ee5c1fb8cf</anchor>
      <arglist>(const std::string &amp;text, TextColor color)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>auto_apms_util::splitString</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga5c77f49068aaa8b8a7f9f4c039359f99</anchor>
      <arglist>(const std::string &amp;str, const std::string &amp;delimiter, bool remove_empty=true)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::printMap</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gae1b8af60a949b81dce119c4f260bc773</anchor>
      <arglist>(const std::map&lt; std::string, std::string &gt; &amp;map, const std::string &amp;key_val_sep=&quot;=&quot;, const std::string &amp;entry_sep=&quot;, &quot;)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::trimWhitespaces</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga297493100cc10a04f7568ec38b26b924</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::toCamelCase</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gaff05f09c20fe90c8c2c605da52e20a22</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::toSnakeCase</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga3e7540f3fcaffba2d61838d39e5f5713</anchor>
      <arglist>(const std::string &amp;str)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>auto_apms_util::contains</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga26d1cc4c19097b3deb2cc7c98b25748c</anchor>
      <arglist>(const ContainerT&lt; ValueT, AllocatorT &gt; &amp;c, const ValueT &amp;val)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; KeyT, CompareT, AllocatorT &gt;</type>
      <name>auto_apms_util::getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8979d95201a925f6524f9413e74b74f8</anchor>
      <arglist>(std::set&lt; KeyT, CompareT, AllocatorT &gt; c1, std::set&lt; KeyT, CompareT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; KeyT, AllocatorT &gt;</type>
      <name>auto_apms_util::getCommonElements</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1e57be4a3189f25e8c145aa93550c07d</anchor>
      <arglist>(std::vector&lt; KeyT, AllocatorT &gt; c1, std::vector&lt; KeyT, AllocatorT &gt; c2)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>auto_apms_util::isFileEmpty</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga52a132b3de3720d7421ce7f49f7b766d</anchor>
      <arglist>(const std::string &amp;path)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>auto_apms_util::exposeToGlobalDebugLogging</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8b5c7b5382f9835da0e937d08a3ae66b</anchor>
      <arglist>(const rclcpp::Logger &amp;logger)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>auto_apms_util::setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga7989c884b003bb1510022b4a1df41bd4</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, const std::string &amp;severity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>auto_apms_util::setLoggingSeverity</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga9e82bc25ea431ee055ef4728ae2d3e9c</anchor>
      <arglist>(const rclcpp::Logger &amp;logger, rclcpp::Logger::Level severity)</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>auto_apms_util::getPackagesWithResourceType</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1aa65bc57e6001b9a6254280bade88ef</anchor>
      <arglist>(const std::string &amp;resource_type, const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::set&lt; std::string &gt;</type>
      <name>auto_apms_util::getPackagesWithPluginResources</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>gabbaa3e1d45d5733e8348af54b61841fa</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="function">
      <type>std::string</type>
      <name>auto_apms_util::getPluginXMLPath</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga5895aab44d575c01e18de138c9ad0992</anchor>
      <arglist>(const std::string &amp;package)</arglist>
    </member>
    <member kind="function">
      <type>std::vector&lt; std::string &gt;</type>
      <name>auto_apms_util::collectPluginXMLPaths</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga8aef66ef74d7e817b3dba660a2ab3bbe</anchor>
      <arglist>(const std::set&lt; std::string &gt; &amp;exclude_packages={})</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS</name>
      <anchorfile>group__auto__apms__util.html</anchorfile>
      <anchor>ga1a7a1c5c9de6370beebb4b36f87c7179</anchor>
      <arglist>(ClassType)</arglist>
    </member>
  </compound>
</tagfile>
