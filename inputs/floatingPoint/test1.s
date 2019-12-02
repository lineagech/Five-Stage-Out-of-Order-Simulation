.text

	# Store word aligned
	# Store 0.5 into 0x10000000
        sw $12, 0($10)

        # Store word also aligned
        # 0.25 into 0x10000004
        sw $13, 3($11)

        # Load word aligned
        lwc1 $f0, 0($10)

        # Load word also aligned
        lwc1 $f1, 3($11)

        add.s $f2, $f0, $f1
        # $f2 = 0.5 + 0.25 = 0.75
        sub.s $f3, $f0, $f1
        # $f3 = 0.5 - 0.25 = 0.25
        sub.s $f4, $f1, $f0
        # $f4 = 0.25 - 0.5 = -0.25
        mul.s $f5, $f0, $f1
        # $f5 = 0.5 * 0.25 = 0.125
        div.s $f6, $f0, $f1
        # $f6 = 0.5 / 0.25 = 2
        div.s $f7, $f1, $f0
        # $f7 = 0.25 / 0.5 = 0.5

        swc1 $f2, 8($10)
        swc1 $f3, 12($10)
        swc1 $f4, 16($10)
        swc1 $f5, 20($10)
        swc1 $f6, 24($10)
        swc1 $f7, 28($10)

	addiu $2, $0, 10
	syscall
