#ifndef __FC_UTILS_H__
#define __FC_UTILS_H__


namespace fc
{


/**
 * \brief Returns the value of a parameter or a default value.
 *
 * \param key the key of the parameter
 * \param default_value the default value to use in case the parameter
 *      does not exist
 * \return value of the parameter
 */
template<typename T>
T get_param(std::string const& key, T default_value)
{
    static ros::NodeHandle node("~");

    auto value = T{default_value};
    if(node.hasParam(key))
    {
        if(!node.getParam(key, value))
        {
            ROS_FATAL_STREAM(
                    "Param with name " << key << " exists but could not be read."
            );
        }
    }

    ROS_INFO_STREAM(
            "Parameter: " << key << " = " << value <<
            " ( " << default_value << " )"
    );

    return value;
}


} /* namespace fc */


#endif
