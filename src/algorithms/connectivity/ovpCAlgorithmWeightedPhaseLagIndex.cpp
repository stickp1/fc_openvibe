#if defined(TARGET_HAS_ThirdPartyEIGEN)

#include "ovpCAlgorithmWeightedPhaseLagIndex.h"
#include <cmath>
#include <complex>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <iostream>



using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessing;

using namespace Eigen;
using namespace std;


boolean CAlgorithmWeightedPhaseLagIndex::initialize(void)
{

	ip_pSignal1.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1));
	ip_pSignal2.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2));

	ip_pChannelPairs.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix));
	op_pMatrix.initialize(this->getOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix));

	// Create algorithm instance of Hilbert transform
	m_pHilbertTransform = &this->getAlgorithmManager().getAlgorithm(this->getAlgorithmManager().createAlgorithm(OVP_ClassId_Algorithm_HilbertTransform));
	m_pHilbertTransform->initialize();

	ip_pHilbertInput.initialize(m_pHilbertTransform->getInputParameter(OVP_Algorithm_HilbertTransform_InputParameterId_Matrix));
	op_pHilbertMatrix.initialize(m_pHilbertTransform->getOutputParameter(OVP_Algorithm_HilbertTransform_OutputParameterId_HilbertMatrix));

	return true;
}

boolean CAlgorithmWeightedPhaseLagIndex::uninitialize(void) {

	ip_pSignal1.uninitialize();
	ip_pSignal2.uninitialize();

	ip_pHilbertInput.uninitialize();
	op_pHilbertMatrix.uninitialize();

	ip_pChannelPairs.uninitialize();
	op_pMatrix.uninitialize();

	m_pHilbertTransform->uninitialize();
	this->getAlgorithmManager().releaseAlgorithm(*m_pHilbertTransform);

	return true;
}


boolean CAlgorithmWeightedPhaseLagIndex::process(void)
{
	const std::complex <double> iComplex(0.0,1.0);

	IMatrix* l_pInputMatrix1 = ip_pSignal1;
	IMatrix* l_pInputMatrix2 = ip_pSignal2;

	IMatrix* l_pChannelPairs = ip_pChannelPairs;
	IMatrix* l_pOutputMatrix = op_pMatrix;

	uint32 l_ui32ChannelCount1 = l_pInputMatrix1->getDimensionSize(0);
	uint32 l_ui32SamplesPerChannel1 = l_pInputMatrix1->getDimensionSize(1);

	uint32 l_ui32SamplesPerChannel2 = l_pInputMatrix2->getDimensionSize(1);

	uint32 l_ui32PairsCount = ip_pChannelPairs->getDimensionSize(0)/2;

	MatrixXd l_pChannelToCompare = MatrixXd::Zero(ip_pChannelPairs->getDimensionSize(0),l_ui32SamplesPerChannel1);

	float64* l_ipMatrixBuffer1 = l_pInputMatrix1->getBuffer();
	float64* l_ipMatrixBuffer2 = l_pInputMatrix2->getBuffer();
	float64* l_opMatrixBuffer = l_pOutputMatrix->getBuffer();

	float64* l_pHilberInputBuffer = ip_pHilbertInput->getBuffer();


	if(this->isInputTriggerActive(OVP_Algorithm_Connectivity_InputTriggerId_Initialize))
	{


		if(l_ui32SamplesPerChannel1 != l_ui32SamplesPerChannel2)
		{
			this->getLogManager() << LogLevel_Error << "Can't compute wPLI on two signals with different lengths";
			return false;
		}

		if(l_ui32SamplesPerChannel1<2||l_ui32SamplesPerChannel2<2)
		{
			uint32 l_size = l_ui32SamplesPerChannel1<=l_ui32SamplesPerChannel2?l_ui32SamplesPerChannel1:l_ui32SamplesPerChannel2;
			this->getLogManager() << LogLevel_Error << "Can't compute wPLI, input signal size = "<<l_size<<"\n";
			return false;
		}

		// Setting size of output
		l_pOutputMatrix->setDimensionCount(2); // the output matrix will have 2 dimensions
		l_pOutputMatrix->setDimensionSize(0,l_ui32PairsCount); //
		l_pOutputMatrix->setDimensionSize(1,1);//

		// Setting name of output channels for visualization
		CString l_name1, l_name2, l_name;
		uint32 l_ui32Index;
		for(uint32 i=0;i<l_ui32PairsCount;i++)
		{
			l_ui32Index=2*i;
			l_name1 = l_pInputMatrix1->getDimensionLabel(0,(uint32)(l_pChannelPairs->getBuffer()[l_ui32Index]));
			l_name2 = l_pInputMatrix2->getDimensionLabel(0,(uint32)(l_pChannelPairs->getBuffer()[l_ui32Index+1]));
			l_name = l_name1+l_name2;
			l_pOutputMatrix->setDimensionLabel(0,i,l_name);
		}

		//
		ip_pHilbertInput->setDimensionCount(2);
		ip_pHilbertInput->setDimensionSize(0,1);
		ip_pHilbertInput->setDimensionSize(1,l_ui32SamplesPerChannel1);

	}

	if(this->isInputTriggerActive(OVP_Algorithm_Connectivity_InputTriggerId_Process))
	{

		VectorXd l_vecXdChannelToCompare1;
		VectorXd l_vecXdChannelToCompare2;
		VectorXd l_vecXdAnalytic1;
		VectorXd l_vecXdAnalytic2;

		std::complex <double> l_sum(0.0,0.0);

		//_______________________________________________________________________________________
		//
		// Compute WPLI for each pairs
		//_______________________________________________________________________________________
		//

		for(uint32 channel = 0; channel < l_ui32PairsCount; channel++)
		{
			l_vecXdChannelToCompare1 = VectorXd::Zero(l_ui32SamplesPerChannel1);
			l_vecXdChannelToCompare2 = VectorXd::Zero(l_ui32SamplesPerChannel2);
			l_vecXdAnalytic1 = VectorXd::Zero(l_ui32SamplesPerChannel1);
			l_vecXdAnalytic2 = VectorXd::Zero(l_ui32SamplesPerChannel2);
			l_sum = 0.0;		// imaginary part is set to zero by definition

			uint32 l_channelIndex = 2*channel; //Index on single channel

			//_______________________________________________________________________________________
			//
			// Form pairs with the lookup matrix given
			//_______________________________________________________________________________________
			//

			for(uint32 sample = 0; sample < l_ui32SamplesPerChannel1; sample++)
			{
				if(l_pChannelPairs->getBuffer()[sample] < l_ui32ChannelCount1)
				{
					l_pChannelToCompare(l_channelIndex,sample) = l_ipMatrixBuffer1[sample+(uint32)l_pChannelPairs->getBuffer()[l_channelIndex]*l_ui32SamplesPerChannel1];
					l_pChannelToCompare(l_channelIndex+1,sample) = l_ipMatrixBuffer2[sample+(uint32)l_pChannelPairs->getBuffer()[l_channelIndex+1]*l_ui32SamplesPerChannel2];
				}
			}


			// Retrieve the 2 channel to compare
			l_vecXdChannelToCompare1 = l_pChannelToCompare.row(l_channelIndex);
			l_vecXdChannelToCompare2 = l_pChannelToCompare.row(l_channelIndex+1);

			// Apply Hilbert transform to each channel to compute instantaneous phase
				// Channel 1
			for(uint32 sample = 0; sample<l_ui32SamplesPerChannel1; sample++)
			{
				l_pHilberInputBuffer[sample] = l_vecXdChannelToCompare1(sample); // Pass channel 1 as input for the Hilbert transform algorithm
			}


			if(m_pHilbertTransform->process(OVP_Algorithm_HilbertTransform_InputTriggerId_Initialize)) // Check initialization before doing the process on channel 1
			{
				m_pHilbertTransform->process(OVP_Algorithm_HilbertTransform_InputTriggerId_Process);

				// Channel 2
				for(uint32 i=0; i<l_ui32SamplesPerChannel1; i++)
				{
					l_vecXdAnalytic1(i) = op_pHilbertMatrix->getBuffer()[i]; // Store imaginary part given by Hilbert algorithm for analytic representation
					l_pHilberInputBuffer[i] = l_vecXdChannelToCompare2(i); // Pass channel 2 as input for the Hilbert transform algorithm
				}

				if(m_pHilbertTransform->process(OVP_Algorithm_HilbertTransform_InputTriggerId_Initialize)) // Check initialization before doing the process on channel 2
				{
					m_pHilbertTransform->process(OVP_Algorithm_HilbertTransform_InputTriggerId_Process);

					// Compute WPLI and store it in l_opMatrixBuffer for each pair
					
					complex<double> num;
					complex<double> norm;

					for(uint32 i=0; i<l_ui32SamplesPerChannel1; i++)
					{
						l_vecXdAnalytic2(i) = op_pHilbertMatrix->getBuffer()[i]; // Store instantaneous phase given by Hilbert algorithm
						
						num += (l_vecXdChannelToCompare1(i)+iComplex*l_vecXdAnalytic1(i))*(l_vecXdChannelToCompare2(i)-iComplex*l_vecXdAnalytic2(i)); // cross-spectrum where spectrum is obtained through Hilbert (real signal + imaginary part) 
						norm += abs((l_vecXdChannelToCompare1(i)+iComplex*l_vecXdAnalytic1(i))*(l_vecXdChannelToCompare2(i)-iComplex*l_vecXdAnalytic2(i))); // normalization factor. If sgn(num) is always 1 or -1, WPLI yields maximum connectivity
					}
					l_opMatrixBuffer[channel] = abs(imag(num)/norm);
					//cout<<"wPLI: "<<l_opMatrixBuffer[channel]<<endl;
				}

			}

			else // Display error if initialization of Hilbert transform was unsuccessful
			{
				this->getLogManager() << LogLevel_Error << "Hilbert transform initialization returned bad status\n";
			}

		}

		this->activateOutputTrigger(OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone, true);
	}

	return true;
}


#endif //TARGET_HAS_ThirdPartyEIGEN
