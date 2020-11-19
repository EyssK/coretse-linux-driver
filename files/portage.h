
#ifndef PORTAGE_H
#define PORTAGE_H


#define NULL_POINTER                            0

#define MAC_ADDRESS_LENGTH                      6u

#define DUPLEX_MODE_MASK                        0x01u

#define BROADCAST_COMPARE_BITS                  0xFFu
#define MULTICAST_ADDRESS_MASK                  0x01u
#define MULTICAST_ADDRESS                       0x01u
#define UNICAST_ADDRESS                         0x00u

#if !defined(NDEBUG)

#define IS_WORD_ALIGNED(x)                      ((uint32_t)0 ==\
                                                ((uint32_t)x & (uint32_t)3))

#define IS_STATE(x)                             (((x) == TSE_ENABLE) ||\
                                                ((x) == TSE_DISABLE))
#define HAL_ASSERT(CHECK) WARN_ON(CHECK)

#elif /* NDEBUG */

#define HAL_ASSERT(CHECK)
#define IS_STATE(x)
#define IS_WORD_ALIGNED(x)

#endif /* NDEBUG */

#define INVALID_INDEX                           (-1)
/***************************************************************************//**
 */
#define FIELD_OFFSET(FIELD_NAME)  (FIELD_NAME##_OFFSET)
#define FIELD_SHIFT(FIELD_NAME)   (FIELD_NAME##_SHIFT)
#define FIELD_MASK(FIELD_NAME)    (FIELD_NAME##_MASK)

/***************************************************************************//**
 * The macro HAL_set_32bit_reg() allows writing a 32 bits wide register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to write. These strings are
 *              specified in a header file associated with the peripheral.
 * VALUE:       A variable of type uint32_t containing the value to write.
 */
#define HAL_set_32bit_reg(BASE_ADDR, REG_NAME, VALUE) \
            (csrwr32((VALUE), (BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 * The macro HAL_get_32bit_reg() is used to read the value  of a 32 bits wide
 * register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to read. These strings are
 *              specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint32_t value.
 */
#define HAL_get_32bit_reg(BASE_ADDR, REG_NAME) \
            (csrrd32((BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 * The macro HAL_set_32bit_reg_field() is used to write a field within a
 * 32 bits wide register. The field written can be one or more bits.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * FIELD_NAME:  A string identifying the register field to write. These strings
 *              are specified in a header file associated with the peripheral.
 * VALUE:       A variable of type uint32_t containing the field value to write.
 */
#define HAL_set_32bit_reg_field(BASE_ADDR, FIELD_NAME, VALUE) \
            do { \
                u32 _value = csrrd32((BASE_ADDR), FIELD_OFFSET(FIELD_NAME));    \
                _value &= (~FIELD_MASK(FIELD_NAME)) | ((VALUE) << FIELD_SHIFT(FIELD_NAME));                            \
                csrwr32(_value, (BASE_ADDR), FIELD_OFFSET(FIELD_NAME));         \
            } while (0)

/***************************************************************************//**
 * The macro HAL_get_32bit_reg_field() is used to read a register field from
 * within a 32 bit wide peripheral register. The field can be one or more bits.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * FIELD_NAME:  A string identifying the register field to write. These strings
 *              are specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint32_t value.
 */
#define HAL_get_32bit_reg_field(BASE_ADDR, FIELD_NAME) \
            ((csrrd32((BASE_ADDR), FIELD_OFFSET(FIELD_NAME)) & FIELD_MASK(FIELD_NAME)) >> FIELD_SHIFT(FIELD_NAME))


/***************************************************************************//**
 * The macro HAL_set_16bit_reg() allows writing a 16 bits wide register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to write. These strings are
 *              specified in a header file associated with the peripheral.
 * VALUE:       A variable of type uint_fast16_t containing the value to write.
 */
#define HAL_set_16bit_reg(BASE_ADDR, REG_NAME, VALUE) \
            (csrwr16((VALUE), (BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 * The macro HAL_get_16bit_reg() is used to read the value  of a 16 bits wide
 * register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to read. These strings are
 *              specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint16_t value.
 */
#define HAL_get_16bit_reg(BASE_ADDR, REG_NAME) \
            (csrrd16((BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 * The macro HAL_set_16bit_reg_field() is used to write a field within a
 * 16 bits wide register. The field written can be one or more bits.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * FIELD_NAME:  A string identifying the register field to write. These strings
 *              are specified in a header file associated with the peripheral.
 * VALUE:       A variable of type uint16_t containing the field value to write.
 */
#define HAL_set_16bit_reg_field(BASE_ADDR, FIELD_NAME, VALUE) \
            do { \
                u16 _value = csrrd16((BASE_ADDR), FIELD_OFFSET(FIELD_NAME));    \
                _value &= (~FIELD_MASK(FIELD_NAME)) | ((VALUE) << FIELD_SHIFT(FIELD_NAME));                            \
                csrwr16(_value, (BASE_ADDR), FIELD_OFFSET(FIELD_NAME));         \
            } while (0)

/***************************************************************************//**
 * The macro HAL_get_16bit_reg_field() is used to read a register field from
 * within a 8 bit wide peripheral register. The field can be one or more bits.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * FIELD_NAME:  A string identifying the register field to write. These strings
 *              are specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint16_t value.
 */
#define HAL_get_16bit_reg_field(BASE_ADDR, FIELD_NAME) \
            ((csrrd16((BASE_ADDR), FIELD_OFFSET(FIELD_NAME)) & FIELD_MASK(FIELD_NAME)) >> FIELD_SHIFT(FIELD_NAME))

/***************************************************************************//**
 * The macro HAL_set_8bit_reg() allows writing a 8 bits wide register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to write. These strings are
 *              specified in a header file associated with the peripheral.
 * VALUE:       A variable of type uint_fast8_t containing the value to write.
 */
#define HAL_set_8bit_reg(BASE_ADDR, REG_NAME, VALUE) \
            (csrwr6((VALUE), (BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 * The macro HAL_get_8bit_reg() is used to read the value of a 8 bits wide
 * register.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * REG_NAME:    A string identifying the register to read. These strings are
 *              specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint8_t value.
 */
#define HAL_get_8bit_reg(BASE_ADDR, REG_NAME) \
            (csrrd8((BASE_ADDR), (REG_NAME##_REG_OFFSET)))

/***************************************************************************//**
 */
#define HAL_set_8bit_reg_field(BASE_ADDR, FIELD_NAME, VALUE) \
            do { \
                u8 _value = csrrd8((BASE_ADDR), FIELD_OFFSET(FIELD_NAME));    \
                _value &= (~FIELD_MASK(FIELD_NAME)) | ((VALUE) << FIELD_SHIFT(FIELD_NAME));                            \
                csrwr8(_value, (BASE_ADDR), FIELD_OFFSET(FIELD_NAME));         \
            } while (0)

/***************************************************************************//**
 * The macro HAL_get_8bit_reg_field() is used to read a register field from
 * within a 8 bit wide peripheral register. The field can be one or more bits.
 *
 * BASE_ADDR:   A variable of type addr_t specifying the base address of the
 *              peripheral containing the register.
 * FIELD_NAME:  A string identifying the register field to write. These strings
 *              are specified in a header file associated with the peripheral.
 * RETURN:      This function-like macro returns a uint8_t value.
 */
#define HAL_get_8bit_reg_field(BASE_ADDR, FIELD_NAME) \
            ((csrrd8((BASE_ADDR), FIELD_OFFSET(FIELD_NAME)) & FIELD_MASK(FIELD_NAME)) >> FIELD_SHIFT(FIELD_NAME))

#endif /*PORTAGE_H*/
